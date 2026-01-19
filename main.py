import math
import time
import matplotlib.pyplot as plt
import csv
import random

# Waypoints (targets)
waypoints = [(10, 0), (10, 12), (1,4), (0, 0)]

#obstacles (x,y,radius)
obstacles = [(6.0, 0.0, 1.0), (10.0, 8.0, 1.0), (4.0, 7.0, 1.0)]

safety_margin = 0.3  # extra buffer around obstacle (avoid earlier)

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

# -----------------------
# GPS settings
# -----------------------
gps_sigma = 0.15          # noise std-dev in "world units" (tune: 0.05..0.30)
gps_dropout_prob = 0.02   # 2% chance gps fails each tick (tune: 0.0..0.10)
gps_x, gps_y = x, y       # last known GPS measurement

def read_gps(true_x, true_y, last_gps_x, last_gps_y, sigma, dropout_prob):
    """
    Simulates a GPS sensor.
    - Usually returns true position + noise.
    - Sometimes "drops out" and returns the last measurement (hold-last-value behavior).
    """
    if random.random() < dropout_prob:
        return last_gps_x, last_gps_y, True  # dropped out

    meas_x = true_x + random.gauss(0.0, sigma)
    meas_y = true_y + random.gauss(0.0, sigma)
    return meas_x, meas_y, False

# -----------------------
# LIDAR-lite settings
# -----------------------
lidar_num_rays = 21
lidar_fov_deg = 120.0                 # forward field-of-view
lidar_max_range = 4.0                 # how far the sensor can see
lidar_sigma = 0.03                    # range noise (small)
lidar_dropout_prob = 0.01             # per-ray dropout chance


# Store path for drawing
path_x = [x]
path_y = [y]

# Plot setup
plt.ion()  # interactive mode ON
fig, ax = plt.subplots()
ax.set_aspect("equal", adjustable="box")
ax.set_title("Waypoint Autopilot GPS + LIDAR (simple)")
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


# GPS dot - shows jittery measurement
gps_dot, = ax.plot([gps_x], [gps_y], marker="x")

#heading indicator line (shows which way boat points)
heading_len = 1.0
heading_arrow = ax.quiver(
    x, y,
    heading_len, 0.0,           # initially pointing right
    angles="xy", scale_units="xy", scale=1,
    width=0.008, zorder=6
)

# LIDAR ray artists (lines)
ray_lines = []
for _ in range(lidar_num_rays):
    line, = ax.plot([], [], linewidth=1, alpha=0.5)
    ray_lines.append(line)

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



def determine_speed(dist_to_wp, lidar_min_range):
    """
    Speed slowed by:
      - distance to waypoint
      - nearest lidar obstacle range (sensor-based)
    """
    # Waypoint slowdown
    wp_factor = min(1.0, dist_to_wp / slow_thresh)
    wp_factor = max(0.15, wp_factor)

    # Obstacle slowdown based on lidar
    obs_slow_thresh = 2.0     # start slowing if obstacle is within 2 units
    obs_min_factor = 0.05

    if lidar_min_range >= obs_slow_thresh:
        obs_factor = 1.0
    else:
        t = max(0.0, lidar_min_range) / obs_slow_thresh  # 0..1
        obs_factor = obs_min_factor + (1.0 - obs_min_factor) * t

    speed_factor = min(wp_factor, obs_factor)
    step_used = fast_step * speed_factor

    return min(step_used, dist_to_wp)  # clamp overshoot to target



def update_plot(true_x, true_y, gps_x, gps_y, heading, path_x, path_y,
                lidar_angles, lidar_ranges):
    boat_dot.set_data([true_x], [true_y])
    path_line.set_data(path_x, path_y)
    gps_dot.set_data([gps_x], [gps_y])

    # Heading arrow
    u = heading_len * math.cos(heading)
    v = heading_len * math.sin(heading)
    heading_arrow.set_offsets([true_x, true_y])
    heading_arrow.set_UVC(u, v)

    # LIDAR rays (draw from boat out to measured range)
    for line, ang, r in zip(ray_lines, lidar_angles, lidar_ranges):
        x2 = true_x + math.cos(ang) * r
        y2 = true_y + math.sin(ang) * r
        line.set_data([true_x, x2], [true_y, y2])

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


# -----------------------
# Ray / circle intersection
# -----------------------
def ray_circle_hit_distance(px, py, dx, dy, cx, cy, radius):
    """
    Ray: P + t*D (t >= 0), D must be unit vector.
    Returns smallest t that hits circle, or None if no hit.
    """
    fx = px - cx
    fy = py - cy

    b = 2.0 * (fx * dx + fy * dy)
    c = (fx * fx + fy * fy) - radius * radius

    disc = b * b - 4.0 * c  # a = 1 since D is unit length
    if disc < 0.0:
        return None

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / 2.0
    t2 = (-b + sqrt_disc) / 2.0

    # We want the nearest non-negative hit
    if t1 >= 0.0:
        return t1
    if t2 >= 0.0:
        return t2
    return None

def lidar_scan(px, py, heading, obstacles, safety_margin,
              num_rays, fov_deg, max_range, sigma, dropout_prob):
    """
    Returns:
      angles: list of ray world angles
      ranges: list of measured distances (0..max_range)
    Rays are centered on heading across +/- fov/2.
    """
    angles = []
    ranges = []

    if num_rays == 1:
        rel_angles = [0.0]
    else:
        fov_rad = math.radians(fov_deg)
        start = -0.5 * fov_rad
        step = fov_rad / (num_rays - 1)
        rel_angles = [start + k * step for k in range(num_rays)]

    for rel in rel_angles:
        ang = heading + rel
        dx = math.cos(ang)
        dy = math.sin(ang)

        best = None
        for (ox, oy, r) in obstacles:
            inflated_r = r + safety_margin
            t_hit = ray_circle_hit_distance(px, py, dx, dy, ox, oy, inflated_r)
            if t_hit is not None and t_hit <= max_range:
                if best is None or t_hit < best:
                    best = t_hit

        # If no hit, return max_range
        true_range = max_range if best is None else best

        # Dropout -> pretend no hit
        if random.random() < dropout_prob:
            meas = max_range
        else:
            meas = true_range + random.gauss(0.0, sigma)
            meas = max(0.0, min(max_range, meas))

        angles.append(ang)
        ranges.append(meas)

    return angles, ranges

def avoid_vector_from_lidar(angles, ranges, max_range):
    """
    Convert ranges into an avoidance vector.
    - Close hits contribute strong push away.
    - Far/no hits contribute almost nothing.
    """
    ax_avoid = 0.0
    ay_avoid = 0.0

    for ang, r in zip(angles, ranges):
        # Normalize "closeness": 0 far, 1 very close
        closeness = (max_range - r) / max_range
        if closeness <= 0.0:
            continue

        # Stronger as you get closer (square it)
        strength = closeness * closeness

        # Ray points in direction ang, so "away" is opposite
        ax_avoid += -math.cos(ang) * strength
        ay_avoid += -math.sin(ang) * strength

    return ax_avoid, ay_avoid

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
        #GPS read
        gps_x, gps_y, gps_drop = read_gps(x, y, gps_x, gps_y, gps_sigma, gps_dropout_prob)

        tx, ty = waypoints[i]

        #compute guidance using GPS, not truth
        dx = tx - gps_x
        dy = ty - gps_y
        dist = math.hypot(dx, dy)

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
        if dist > 1e-9:
            goal_dx = dx / dist
            goal_dy = dy / dist
        else:
            goal_dx, goal_dy = 0.0, 0.0

        # LIDAR scan (sensor)
        lidar_angles, lidar_ranges = lidar_scan(
            x, y, heading,
            obstacles, safety_margin,
            lidar_num_rays, lidar_fov_deg, lidar_max_range,
            lidar_sigma, lidar_dropout_prob
        )
        lidar_min = min(lidar_ranges) if lidar_ranges else lidar_max_range

         # Turn sensor ranges into avoidance vector
        avoid_x, avoid_y = avoid_vector_from_lidar(lidar_angles, lidar_ranges, lidar_max_range)

        # Blend goal + avoid
        avoid_weight = 6.0   # tune this
        blend_dx = goal_dx + avoid_weight * avoid_x
        blend_dy = goal_dy + avoid_weight * avoid_y

        desired_heading = math.atan2(blend_dy, blend_dx)

        # turn-rate limited heading update (your existing realism constraint)
        heading = update_heading(heading, desired_heading, max_turn_rate, dt)



        #determine speed of boat 
        step_used = determine_speed(dist, lidar_min)
        # Move boat towards target
        x += math.cos(heading) * step_used
        y += math.sin(heading) * step_used
        path_x.append(x)
        path_y.append(y)

        # Update plot
        update_plot(x, y, gps_x, gps_y, heading, path_x, path_y, lidar_angles, lidar_ranges)
        time.sleep(dt)

    #update hud 1 last time to show 4/4
    set_hud(t_s, step_idx, wp_reached, tx, ty, x, y, dist)
    plt.ioff()
    plt.show()

