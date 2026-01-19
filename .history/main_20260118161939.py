import math
import time
import matplotlib.pyplot as plt
import csv

# Waypoints (targets)
waypoints = [(10, 0), (10, 10), (0, 10), (0, 0)]

#obstacles (x,y,radius)
obstacles = [(6.0, 2.0, 1.0), (8.0, 8.0, 1.2), (3.0, 7.0, 0.9)]

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
max_turn_rate_deg = 90.0                            # degrees per second (tune: 30..120)
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


def determine_speed(dist):
    speed_factor = min(1.0, dist / slow_thresh)  # dist=slow_thresh -> 1.0, dist smaller -> <1.0
    speed_factor = max(0.15, speed_factor)       # don't go below 15% speed
    step_used = fast_step * speed_factor
    return min(step_used, dist)     #accounts for "clamp" (prevents overshoot)


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

        #determine where the boat needs to point
        #desired_heading = math.atan2(dy,dx)                                    #angle that points directly towards the target
        desired_x = dx
        desired_y = dy
        heading = update_heading(heading, desired_heading, max_turn_rate, dt)   #angle that it is allowed to point

        #determine speed of boat 
        step_used = determine_speed(dist)
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

