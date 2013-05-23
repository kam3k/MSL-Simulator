# Python imports
import random
from numpy import linspace
from math import sin, cos, sqrt, pi

# MSL Sim imports
from sim.model import lines
import sim.config as c

def _find_intersection(line_1, line_2):
    """Finds the intersection point (x, y) of two infinitely long lines. The 
    lines input to this function are dictionaries with keys a, b, and c that
    describe the line in the form ax + by = c."""
    det = line_1['a'] * line_2['b'] - line_2['a'] * line_1['b']
    # Lines are parallel if the determinant is zero
    if det == 0:
        return None
    x = (line_2['b'] * line_1['c'] - line_1['b'] * line_2['c']) / float(det)
    y = (line_1['a'] * line_2['c'] - line_2['a'] * line_1['c']) / float(det)
    return (x, y)

def _validate_intersection(line, point):
    """Determines whether or not a point that is known to be on a line is on a 
    particular segment of that line. The line dictionary has the endpoints of
    the line segment (keys x_1, y_1, x_2, and y_2)."""
    x, y = point
    if (min(line['x_1'], line['x_2']) <= x <= max(line['x_1'], line['x_2']) and
        min(line['y_1'], line['y_2']) <= y <= max(line['y_1'], line['y_2'])):
        return True
    else:
        return False

def _get_laser_lines(pose):
    """Given the pose of the robot, returns a list of line dictionaries. Each
    dictionary contains information about a single beam in the laser scan."""
    x, y, theta = pose
    laser_lines = []
    for beta in linspace(c.LASER_MIN_ANGLE, c.LASER_MAX_ANGLE, 
            (c.LASER_MAX_ANGLE - c.LASER_MIN_ANGLE)/c.LASER_RES + 1):
        beta = beta * pi/180
        x_2 = x + c.LASER_RANGE * cos(theta + beta)
        y_2 = y + c.LASER_RANGE * sin(theta + beta)
        laser_lines.append(lines.get_line_dict(x, y, x_2, y_2))
    return laser_lines

def _dist_between_points(point_1, point_2):
    """Returns the absolute distance between two points."""
    x_1, y_1 = point_1
    x_2, y_2 = point_2
    return sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)

def _dist_point_to_line(line, point):
    """Returns the minimum distance between a point and a line."""
    x, y = point
    return ( (line['a'] * x + line['b'] * y - line['c']) / 
            sqrt(line['a']**2 + line['b']**2))

def scan(pose, line_map):
    """Given the pose of the robot and a list of line segments (line_map),
    returns a list of range measurements."""
    ranges = []
    laser_lines = _get_laser_lines(pose)
    for beam in laser_lines:
        r_min = 999
        for line in line_map:
            # skip line if it is entirely outside max laser range
            if _dist_point_to_line(line, (pose[0], pose[1])) > c.LASER_RANGE:
                continue
            # find the (x, y) coordinates of intersection (if it exists)
            intersection = _find_intersection(beam, line)
            # if the lines intersect
            if intersection and _validate_intersection(line, intersection):
                # if the intersection is on both the line segments
                if (_validate_intersection(line, intersection) and 
                        _validate_intersection(beam, intersection)):
                    # get a range measurement
                    r_temp = _dist_between_points((pose[0], pose[1]), intersection)
                    # keep it if its in the max range and its the smallest seen yet
                    if r_temp < r_min and r_temp < c.LASER_RANGE:
                        r_min = r_temp + random.gauss(0, c.LASER_NOISE)
        r_min = r_min if r_min < 999 else 0
        ranges.append(r_min)
    return ranges


if __name__ == '__main__':
    from matplotlib import pyplot as plt
    import time

    pose = (0, 0, 0)

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
        ranges = scan(pose, line_map)
    print 'Performed %d scans with %d lines' % (num_scans, num_lines)
    print 'Elapsed time: ', time.time() - start_time

    x_ranges = []
    y_ranges = []
    for (i, r) in enumerate(ranges):
        if r == 0:
            continue
        x_ranges.append(pose[0] + 
                r*cos(pose[2] + pi/180*(c.LASER_MIN_ANGLE + c.LASER_RES*i)))
        y_ranges.append(pose[0] + 
                r*sin(pose[2] + pi/180*(c.LASER_MIN_ANGLE + c.LASER_RES*i)))

    for l in line_map:
        plt.plot([l['x_1'], l['x_2']], [l['y_1'], l['y_2']], 'b')
    plt.plot(pose[0], pose[1], 'gs')
    plt.plot(x_ranges, y_ranges, 'rx')
    plt.xlim(xmin=-0.5)
    plt.axis('equal')
    plt.show()
