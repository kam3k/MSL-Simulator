import random
import sys
import math as m
import numpy as np

def generate_landmark_corners(x_centre, y_centre, angle, edge_length):
    d = edge_length
    R = np.array([[m.cos(angle), m.sin(angle)], [-m.sin(angle), m.cos(angle)]])
    corner_points = []
    for (x_0, y_0) in [(d/2.0, d/2.0), (d/2.0, -d/2.0), (-d/2.0, -d/2.0), (-d/2.0, d/2.0)]:
        p = np.dot(R, np.array([x_0, y_0])) + np.array([x_centre, y_centre])
        corner_points.append((p[0], p[1]))
    return corner_points

# Set some constants
NUM_LANDMARKS = 100
MIN_SIZE = 0.3 # [m]
MAX_SIZE = 2 # [m]
MIN_X = -30 # [m]
MAX_X = 30 # [m]
MIN_Y = -30 # [m]
MAX_Y = 30 # [m]

# Get the name of the output file
filename = sys.argv[1]

# Generate random squares/diamonds
lines = []
for i in range(NUM_LANDMARKS):
    x_centre = random.uniform(MIN_X, MAX_X)
    y_centre = random.uniform(MIN_Y, MAX_Y)
    edge_length = random.uniform(MIN_SIZE, MAX_SIZE)
    angle = random.uniform(0, m.pi/2)
    corner_points = generate_landmark_corners(x_centre, y_centre, angle, edge_length)
    for i, point in enumerate(corner_points):
        lines.append((corner_points[i-1][0], corner_points[i-1][1], point[0], point[1]))

# Write the lines to a file
with open(filename, 'w+') as f:
    for line in lines:
        f.write('%0.2f %0.2f %0.2f %0.2f\n' % (line[0], line[1], line[2], line[3]))
