import math
import time
import matplotlib.pyplot as plt
import csv

# 1) Waypoints (targets)
waypoints = [(10, 0), (10, 10), (0, 10), (0, 0)]

# 2) Boat starting position
x, y = 0.0, 0.0

# Simulation settings
step = 0.15            # how far the boat moves each tick
threshold = 0.25       # how close counts as "reached waypoint"
dt = 0.05              # seconds per tick (controls animation speed)

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
        tx, ty = waypoints[i]
        dx = tx - x
        dy = ty - y
        dist = math.sqrt(dx*dx + dy*dy)

        # LOG ONE ROW PER LOOP
        t_s = time.time() - start_time
        writer.writerow([t_s, step_idx, i, tx, ty, x, y, dx, dy, dist])
        step_idx += 1

        # If close enough, switch to next waypoint
        if dist < threshold:
            i += 1
            continue

        # Compute a unit direction vector toward the waypoint
        ux = dx / dist
        uy = dy / dist

        # Move one step toward the waypoint
        x += ux * step
        y += uy * step

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
