import math
import time
import matplotlib.pyplot as plt
import csv

# Waypoints (targets)
waypoints = [(10, 0), (10, 10), (1,4), (0, 0)]

#obstacles (x,y,radius)
obstacles = [(6.0, 0.0, 1.0), (10.0, 8.0, 1.0), (4.0, 7.0, 1.0)]

safety_margin = 0.6  # extra buffer around obstacle (avoid earlier)

# Boat starting position
x, y = 0.0, 0.0

# Simulation settings
fast_step = 0.15            #speed at which boat moves (default)
threshold = 0.25            # how close counts as "reached waypoint"
dt = 0.05                   # seconds per tick (controls animation speed)
slow_thresh = 2.0           #radius where we need to slow down 

#boat turn settings
heading = 0.0                                       #where the boat currently faces 
max_turn_rate_deg = 160.0                            # degrees per second (tune: 30..120)
max_turn_rate = math.radians(max_turn_rate_deg)     # convert to rad/s

# Store path for drawing
path_x = [x]
path_y = [y]

# Plot setup
plt.ion()  # interactive mode ON
fig, ax = plt.subplots()
ax.set_aspect("equal", adjustable="box")
ax.set_title("Waypoint Autopilot (simple)")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Draw waypoints
wx = [p[0] for p in waypoints]
wy = [p[1] for p in waypoints]
ax.scatter(wx, wy, marker="o")

#Draw obstacles
for (ox, oy, r) in obstacles:
    circle = plt.Circle((ox, oy), r, fill=False, linewidth=2)
    ax.add_patch(circle)
    # optional: draw the safety margin too (dashed)
    safe_circle = plt.Circle((ox, oy), r + safety_margin, fill=False, linestyle="--", alpha=0.5)
    ax.add_patch(safe_circle)

# Boat + path artists (things we update)
boat_dot, = ax.plot([x], [y], marker="o")
path_line, = ax.plot(path_x, path_y)

#heading indicator line (shows which way boat points)
heading_len = 1.0
heading_arrow = ax.quiver(
    x, y,
    heading_len, 0.0,           # initially pointing right
    angles="xy", scale_units="xy", scale=1,
    width=0.008, zorder=6
)

# Make the view a bit larger than waypoints
margin = 2
ax.set_xlim(min(wx) - margin, max(wx) + margin)
ax.set_ylim(min(wy) - margin, max(wy) + margin)

# ---- HUD TEXT SETUP ----
# Place text in the top-left of the plot using axes coordinates (0..1)
hud = ax.text(
    -0.5, 0.98, "",
    transform=ax.transAxes,
    va="top", ha="left",
    fontsize=10,
    family="monospace",
    bbox=dict(boxstyle="round", alpha=0.8)
)
def set_hud(t_s, step_idx, wp_reached, tx, ty, x, y, dist):
    hud.set_text(
        f"time(s):       {t_s:6.2f}\n"
        f"step indx:  {step_idx}\n"
        f"target:        {wp_reached}/{len(waypoints)}\n"
        f"target indx:    ({tx:5.2f}, {ty:5.2f})\n"
        f"curr pos:       ({x:5.2f}, {y:5.2f})\n"
        f"distance to next target:      {dist:6.3f}\n"
    )

def log_csv(t_s, step_idx, i, tx, ty, x, y, dx, dy, dist):
    writer.writerow([t_s, step_idx, i, tx, ty, x, y, dx, dy, dist])
    reached_wp = 1 if dist < threshold else 0

def nearest_obstacle_surface_dist(x, y, obstacles):
    """
    Returns the smallest distance from the boat to any obstacle surface.
    Positive = outside obstacle
    0 = exactly on the circle
    Negative = inside obstacle
    """
    best = float("inf")
    for (ox, oy, r) in obstacles:
        d_center = math.hypot(x - ox, y - oy)
        surface_dist = d_center - r
        if surface_dist < best:
            best = surface_dist
    return best


def determine_speed(dist_to_wp, x, y, obstacles):
    # --- Waypoint slowdown (your current logic) ---
    wp_factor = min(1.0, dist_to_wp / slow_thresh)
    wp_factor = max(0.15, wp_factor)   # min 15% speed

    # --- Obstacle slowdown (NEW) ---
    obs_surface_dist = nearest_obstacle_surface_dist(x, y, obstacles)

    obs_slow_thresh = 2.0      # start slowing when within 2 units of obstacle surface
    obs_min_factor = 0.05      # can go slower near obstacles than near waypoints

    # If far from obstacles, factor = 1.0. If near, ramps down toward obs_min_factor.
    if obs_surface_dist >= obs_slow_thresh:
        obs_factor = 1.0
    else:
        # map surface_dist from [0..obs_slow_thresh] to [0..1]
        t = max(0.0, obs_surface_dist) / obs_slow_thresh  # clamp negative to 0
        # t=1 -> far (factor 1), t=0 -> at surface (min)
        obs_factor = obs_min_factor + (1.0 - obs_min_factor) * t

    # --- Combine: take the smaller (more cautious) factor ---
    speed_factor = min(wp_factor, obs_factor)

    step_used = fast_step * speed_factor

    # overshoot clamp relative to waypoint
    return min(step_used, dist_to_wp)



def update_plot(x, y, heading, path_x, path_y):
    #update boat and trail
    boat_dot.set_data([x], [y])
    path_line.set_data(path_x, path_y)

    # update heading arrow (u, v is the arrow direction vector)
    u = heading_len * math.cos(heading)
    v = heading_len * math.sin(heading)
    heading_arrow.set_offsets([x, y])  # move arrow base to boat position
    heading_arrow.set_UVC(u, v)        # set arrow direction

    fig.canvas.draw()
    fig.canvas.flush_events()


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def update_heading(current_heading, desired_heading, max_turn_rate_rad_s, dt_s):
    error = wrap_to_pi(desired_heading - current_heading)
    max_delta = max_turn_rate_rad_s * dt_s
    # clamp how much we can change heading this tick
    if error > max_delta:
        error = max_delta
    elif error < -max_delta:
        error = -max_delta
    return wrap_to_pi(current_heading + error)

def compute_avoid_vector(x, y, obstacles, safety_margin):
    ax_avoid = 0.0
    ay_avoid = 0.0

    for (ox, oy, r) in obstacles:
        dxo = x - ox
        dyo = y - oy
        d = math.hypot(dxo, dyo)

        # Distance from boat to the obstacle *surface* (not the center)
        surface_dist = d - r

        # If we're within the safety margin of the surface, push away
        if surface_dist < safety_margin and d > 1e-6:
            # closeness: 0 at edge of safety band, 1 at obstacle surface
            closeness = (safety_margin - surface_dist) / safety_margin

            # stronger ramp as you get closer
            strength = closeness ** 2  # try **3 if you want more aggressive

            # unit vector away from obstacle center (same direction as away from surface for circles)
            ux = dxo / d
            uy = dyo / d

            ax_avoid += ux * strength
            ay_avoid += uy * strength

    return ax_avoid, ay_avoid




start_time = time.time()
step_idx = 0
wp_reached = 0

with open("run_log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "t_s", "step_idx", "wp_index",
        "target_x", "target_y",
        "x", "y",
        "dx", "dy", "dist"
    ])

    i = 0  # waypoint index
    while i < len(waypoints):
        tx, ty = waypoints[i]
        dx = tx - x
        dy = ty - y
        dist = math.sqrt(dx*dx + dy*dy)
        t_s = time.time() - start_time

        #log one row per loop (csv)
        log_csv(t_s, step_idx, i, tx, ty, x, y, dx, dy, dist)
        step_idx += 1
        # update hud box
        set_hud(t_s, step_idx, wp_reached, tx, ty, x, y, dist)

        # If close enough, switch to next waypoint
        if dist < threshold:
            i += 1
            wp_reached+=1
            continue

        # goal directio0n
        goal_dx = dx/dist
        goal_dy = dy/dist
        # avoid direction (away from obstacles)
        avoid_x, avoid_y = compute_avoid_vector(x, y, obstacles, safety_margin)
        # Blend them:
        avoid_weight = 9.0   # tune: bigger = stronger avoidance
        blend_dx = goal_dx + (avoid_weight * avoid_x)
        blend_dy = goal_dy + (avoid_weight * avoid_y)

        desired_heading = math.atan2(blend_dy, blend_dx)

        # turn-rate limited heading update (your existing realism constraint)
        heading = update_heading(heading, desired_heading, max_turn_rate, dt)




        #determine speed of boat 
        step_used = determine_speed(dist, x, y, obstacles)
        # Move boat towards target
        x += math.cos(heading) * step_used
        y += math.sin(heading) * step_used
        path_x.append(x)
        path_y.append(y)

        # Update plot
        update_plot(x, y, heading, path_x, path_y)
        time.sleep(dt)

    #update hud 1 last time to show 4/4
    set_hud(t_s, step_idx, wp_reached, tx, ty, x, y, dist)
    plt.ioff()
    plt.show()

