import math
import time
import matplotlib.pyplot as plt

#waypoints (targets)
waypoints = [(10,0), (10,10), (0,10), (0,0)]

#Boat starting position
x, y = 0.0, 0.0

#Simulation settings
step = 0.15         #how far the boat moves every iteration. bigger step = moves further (but less smooth + can overshoot target)
threshold = 0.25        #how close we must get to the waypoint
dt =                #