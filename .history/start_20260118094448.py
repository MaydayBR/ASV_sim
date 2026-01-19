import math
import time
import matplotlib.pyplot as plt

#waypoints (targets)
waypoints = [(10,0), (10,10), (0,10), (0,0)]

#Boat starting position
x, y = 0.0, 0.0

#Simulation settings
step = 0.15                 #how far the boat moves every iteration. bigger step = moves further (but less smooth + can overshoot target)
threshold = 0.25            #how close we must get to the waypoint
dt = 0.05                   #how long to pause each loop for the animation (smaller = faster animation)

#Boat path (so we can draw the path)
path_x = [x]
path_y = [y]

#plot setup (create the window once)
plt.ion()
fig, ax = plt.subplots()
ax.set_aspect("equal", adjustable="box")
ax.set_title("Waypoint Autopilot (simple)")
ax.set_xlabel("X")
ax.set_ylabel("Y")

#draw the waypoint(target) dots (once)
wx = [p[0] for p in waypoints]
wy = [p[1] for p in waypoints]
ax.scatter(wx, wy, marker="o")

#create artists (these will update every frame)
boat_dot, = ax.plot([x], [y], marker="o")
path_line, = ax.plot(path_x, path_y)

#set the plot boundaries (slightly larger than waypoints)
margin = 2
ax.set_xlim(min(wx) - margin, max(wx) + margin)
ax.set_ylim(min(wy) - margin, max(wy) + margin)


#main loop
i = 0
while i < len(waypoints):
    tx, ty = waypoints[i]
    dx = tx - x
    dy = ty - y
    dist = math.sqrt(dx*dx + dy*dy)

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
