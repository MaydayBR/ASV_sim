import math
import time
import matplotlib.pyplot as plt
import csv

# 1) Waypoints (targets)
waypoints = [(10, 0), (10, 10), (0, 10), (0, 0)]

# 2) Boat starting position
x, y = 0.0, 0.0

# Simulation settings
slow_step = 0.01
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


def determine_speed(dist, slow_thresh):
    if dist < slow_thresh:
        step_used = slow_step
    else:
        step_used = fast_step

    max_allowed = max(0.0, dist - (threshold * 0.5))
    if max_allowed > 0:
        step = min(step_used, max_allowed)
    else:
        # If we're extremely close, just take a very small step
        step = min(step_used, dist)
        
    return step_used 

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
            f"t_s:       {t_s:6.2f}\n"
            f"step_idx:  {step_idx}\n"
            f"wp:        {i+1}/{len(waypoints)}\n"
            f"target:    ({tx:5.2f}, {ty:5.2f})\n"
            f"pos:       ({x:5.2f}, {y:5.2f})\n"
            f"error:     ({dx:5.2f}, {dy:5.2f})\n"
            f"dist:      {dist:6.3f}\n"
            f"reached:   {reached_wp}\n"
            f"dt tgt/act {dt:0.3f}/{dt_actual:0.3f}"
        )

        # If close enough, switch to next waypoint
        if dist < threshold:
            t_s = time.time() - start_time
            writer.writerow([t_s, step_idx, i, tx, ty, x, y, dx, dy, dist, 0.0, 0.0])
            step_idx += 1
            i += 1
            continue

        # Compute a unit direction vector toward the waypoint
        ux = dx / dist
        uy = dy / dist

        #determine speed based on how close we are to the threshold
        step_used = determine_speed(dist, slow_thresh)

        # Move one step toward the waypoint
        x += ux * step_used
        y += uy * step_used


        path_x.append(x)
        path_y.append(y)

        # Update plot
        boat_dot.set_data([x], [y])
        path_line.set_data(path_x, path_y)
        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(dt)

    plt.ioff()
    plt.show()
