import random
import math
import numpy as np
from collections import defaultdict

SLOPE_SET = [34, 97, 2] # degrees
SLOPE_NOISE = 4 # degrees (std. dev)
NUM_SEGMENTS = 50
MIN_LENGTH = 0.2
MAX_LENGTH = 1
START = (-5, -5)

prev_coord = START

line_endpoints = []
slope_info = defaultdict(list)

for seg in range(NUM_SEGMENTS):
    slope = random.choice(SLOPE_SET) 
    noisy_slope = slope + random.gauss(0, SLOPE_NOISE)
    x_1, y_1 = prev_coord
    length = random.uniform(MIN_LENGTH, MAX_LENGTH)
    x_2 = x_1 + length * math.cos(math.radians(noisy_slope))
    y_2 = y_1 + length * math.sin(math.radians(noisy_slope))
    line_endpoints.append((x_1, y_1, x_2, y_2))
    prev_coord = (x_2, y_2)

    slope_info[slope].append((noisy_slope, length))

num_joints = len(slope_info.keys())
joint_length_stats = []
joint_orient_stats = []
for key in slope_info:
    slopes = [i[0] for i in slope_info[key]]
    lengths = [i[1] for i in slope_info[key]]
    joint_orient_stats.append((np.mean(slopes), np.std(slopes)))
    joint_length_stats.append((np.mean(lengths), np.std(lengths)))


with open('maps/random_map.txt', 'w+') as f:
    f.write('### MAP STATISTICS ###\n')
    f.write('# slope slope_std length length_std\n')
    for slopes, lengths in zip(joint_orient_stats, joint_length_stats):
        f.write('# %0.2f %0.4f %0.2f %0.4f\n' % (slopes[0], slopes[1], lengths[0], lengths[1]))
    f.write('### LINE COORDINATES ###\n')
    for points in line_endpoints:
        x_1, y_1, x_2, y_2 = points
        f.write('%0.2f %0.2f %0.2f %0.2f\n' % (x_1, y_1, x_2, y_2))
