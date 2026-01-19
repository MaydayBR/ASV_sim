import math
import time
import matplotlib.pyplot as plt
import csv

# 1) Waypoints (targets)
waypoints = [(10, 0), (10, 10), (0, 10), (0, 0)]

# 2) Boat starting position
x, y = 0.0, 0.0

# Simulation settings
fast_step = 0.15
threshold = 0.25            # how close counts as "reached waypoint"
dt = 0.05                   # seconds per tick (controls animation speed)
slow_thresh = 2.0           #radius where we need to slow down 


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

# Pre-draw waypoints
wx = [p[0] for p in waypoints]
wy = [p[1] for p in waypoints]
ax.scatter(wx, wy, marker="o")  # waypoints

# Boat + path artists (things we update)
boat_dot, = ax.plot([x], [y], marker="o")
path_line, = ax.plot(path_x, path_y)

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


def determine_speed(dist):
    speed_factor = min(1.0, dist / slow_thresh)  # dist=slow_thresh -> 1.0, dist smaller -> <1.0
    speed_factor = max(0.15, speed_factor)       # don't go below 15% speed
    step_used = fast_step * speed_factor
    return step_used

def move_in_x_direction(x, ux, step_used):
    return x + (ux * step_used)

def move_in_y_direction(y, uy, step_used):
    return y + (uy * step_used)

def update_plot(x,y,path_x,path_y):
    boat_dot.set_data([x], [y])
    path_line.set_data(path_x, path_y)
    fig.canvas.draw()
    fig.canvas.flush_events()


start_time = time.time()
step_idx = 0

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
        loop_start = time.time()  # (optional) for dt_actual
        tx, ty = waypoints[i]
        dx = tx - x
        dy = ty - y
        dist = math.sqrt(dx*dx + dy*dy)

        # LOG ONE ROW PER LOOP
        t_s = time.time() - start_time
        writer.writerow([t_s, step_idx, i, tx, ty, x, y, dx, dy, dist])
        step_idx += 1
        reached_wp = 1 if dist < threshold else 0


        # ---- UPDATE HUD (box on the left) ----
        # Round for readability
        dt_actual = time.time() - loop_start
        hud.set_text(
            f"time(s):       {t_s:6.2f}\n"
            f"step indx:  {step_idx}\n"
            f"wp:        {i+1}/{len(waypoints)}\n"
            f"target:    ({tx:5.2f}, {ty:5.2f})\n"
            f"curr pos:       ({x:5.2f}, {y:5.2f})\n"
            f"error:     ({dx:5.2f}, {dy:5.2f})\n"
            f"distance to next target:      {dist:6.3f}\n"
            f"reached:   {reached_wp}\n"
            f"dt tgt/act {dt:0.3f}/{dt_actual:0.3f}"
        )

        # If close enough, switch to next waypoint
        if dist < threshold:
            i += 1
            continue

        # Compute a unit direction vector toward the waypoint
        ux = dx / dist
        uy = dy / dist

        #determine speed based on how close we are to the waypoint (are we inside of slow_thresh?)
        step_used = determine_speed(dist)

        # simple clamp (prevents overshoot)
        step_used = min(step_used, dist)

        # Move one step toward the waypoint
        x = move_in_x_direction(x, ux, step_used)
        y = move_in_y_direction(y, uy, step_used)

        path_x.append(x)
        path_y.append(y)

        # Update plot
        update_plot(x, y, path_x, path_y)
        time.sleep(dt)

    plt.ioff()
    plt.show()
