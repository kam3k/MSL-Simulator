import random
import math
import sys
import numpy as np
from collections import defaultdict


#SLOPE_SET = [0, 90] # degrees
#SLOPE_NOISE = 2 # degrees (std. dev)
#OUTLIER_PERCENT = 0
#NUM_SEGMENTS = 50
SLOPE_SET = [1, 22, 46, 67, 91, 113, 134, 157] # degrees
SLOPE_NOISE = 1 # degrees (std. dev)
OUTLIER_PERCENT = 15
NUM_SEGMENTS = 200
MIN_LENGTH = 0.4
MAX_LENGTH = 1.0
START = (-2, 2)

filename = sys.argv[1]

prev_coord = START

line_endpoints = []
directions = []
slope_info = defaultdict(list)
num_outlier = 0

for seg in range(NUM_SEGMENTS):
    length = random.uniform(MIN_LENGTH, MAX_LENGTH)
    if random.randint(1,100) <= OUTLIER_PERCENT:
        noisy_slope = random.randint(0, 179)
        num_outlier += 1
    else:
        slope = random.choice(SLOPE_SET)
        noisy_slope = slope + random.gauss(0, SLOPE_NOISE)
        slope_info[slope].append((noisy_slope, length))
    x_1, y_1 = prev_coord
    x_2 = x_1 + length * math.cos(math.radians(noisy_slope - 90))
    y_2 = y_1 + length * math.sin(math.radians(noisy_slope - 90))
    line_endpoints.append((x_1, y_1, x_2, y_2))
    directions.append(noisy_slope)
    prev_coord = (x_2, y_2)


num_joints = len(slope_info.keys())
joint_length_stats = []
joint_orient_stats = []
for key in slope_info:
    slopes = [i[0] for i in slope_info[key]]
    lengths = [i[1] for i in slope_info[key]]
    joint_orient_stats.append((np.mean(slopes), np.std(slopes)))
    joint_length_stats.append((np.mean(lengths), np.std(lengths)))


with open(filename, 'w+') as f:
    f.write('### MAP STATISTICS ###\n')
    f.write('# num lines: %d\n' % NUM_SEGMENTS)
    f.write('# num outliers: %d\n' % num_outlier)
    f.write('# slope slope_std length length_std\n')
    for slopes, lengths in zip(joint_orient_stats, joint_length_stats):
        f.write('# %0.2f %0.4f %0.2f %0.4f\n' % (slopes[0], slopes[1], lengths[0], lengths[1]))
    f.write('### LINE COORDINATES ###\n')
    for points, d in zip(line_endpoints, directions):
        x_1, y_1, x_2, y_2 = points
        f.write('%0.2f %0.2f %0.2f %0.2f %0.1f\n' % (x_1, y_1, x_2, y_2, d))
