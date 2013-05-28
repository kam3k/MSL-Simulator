# Python imports
import random
from numpy import linspace
from math import sin, cos, pi

# MSL Sim imports
from sim.model import lines
import sim.defaults as d


class Laser(object):
    def __init__(self, pose):
        self.pose = pose
        self.min_angle = d.LASER_MIN_ANGLE
        self.max_angle = d.LASER_MAX_ANGLE
        self.resolution = d.LASER_RES
        self.range = d.LASER_RANGE
        self.noise = d.LASER_NOISE

    def _get_beam_lines(self):
        """Given the pose of the robot, returns a list of line dictionaries. Each
        dictionary contains information about a single beam in the laser scan."""
        x, y, theta = self.pose
        laser_beams = []
        for beta in linspace(self.min_angle, self.max_angle, 
                (self.max_angle - self.min_angle)/self.resolution + 1):
            beta = beta * pi/180
            x_2 = x + self.range * cos(theta + beta)
            y_2 = y + self.range * sin(theta + beta)
            laser_beams.append(lines.get_line_dict(x, y, x_2, y_2))
        return laser_beams

    def scan(self, line_map):
        """Given the pose of the robot and a list of line segments (line_map),
        returns a list of range measurements."""
        ranges = []
        laser_beams = self._get_beam_lines()
        for beam in laser_beams:
            r_min = 999
            for line in line_map:
                # skip line if it is entirely outside max laser range
                if lines.dist_point_to_line(line, (self.pose[0], self.pose[1])) > self.range:
                    continue
                # find the (x, y) coordinates of intersection (if it exists)
                intersection = lines.find_intersection(beam, line)
                # if the lines intersect
                if intersection and lines.validate_intersection(line, intersection):
                    # if the intersection is on both the line segments
                    if (lines.validate_intersection(line, intersection) and 
                            lines.validate_intersection(beam, intersection)):
                        # get a range measurement
                        r_temp = lines.dist_between_points((self.pose[0], self.pose[1]), intersection)
                        # keep it if its in the max range and its the smallest seen yet
                        if r_temp < r_min and r_temp < self.range:
                            r_min = r_temp + random.gauss(0, self.noise)
            r_min = r_min if r_min < 999 else 0
            ranges.append(r_min)
        return ranges


if __name__ == '__main__':
    from matplotlib import pyplot as plt
    import time

    pose = (0, 0, 0)
    laser = Laser(pose)

    line_map = []

    num_lines = 100
    x = 2
    y = 2
    for l in range(num_lines):
        x_2 = x + random.uniform(-5.0/num_lines, 5.0/num_lines)
        y_2 = y + random.uniform(-5.0/num_lines, -2.0/num_lines)
        li = lines.get_line_dict(x, y, x_2, y_2)
        line_map.append(li)
        x = x_2
        y = y_2

    num_scans = 10

    start_time = time.time()
    for i in range(num_scans):
        ranges = laser.scan(line_map)
    print 'Performed %d scans with %d lines' % (num_scans, num_lines)
    print 'Elapsed time: ', time.time() - start_time

    x_ranges = []
    y_ranges = []
    for (i, r) in enumerate(ranges):
        if r == 0:
            continue
        x_ranges.append(pose[0] + 
                r*cos(pose[2] + pi/180*(laser.min_angle + laser.resolution*i)))
        y_ranges.append(pose[0] + 
                r*sin(pose[2] + pi/180*(laser.min_angle + laser.resolution*i)))

    for l in line_map:
        plt.plot([l['x_1'], l['x_2']], [l['y_1'], l['y_2']], 'b')
    plt.plot(pose[0], pose[1], 'gs')
    plt.plot(x_ranges, y_ranges, 'rx')
    plt.xlim(xmin=-0.5)
    plt.axis('equal')
    plt.show()
