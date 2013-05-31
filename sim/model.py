# Python imports
import random
from numpy import linspace
from math import sin, cos, pi, sqrt

# MSL Sim imports
import sim.defaults as d


def find_intersection(line_1, line_2):
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

def validate_intersection(line, point):
    """Determines whether or not a point that is known to be on a line is on a 
    particular segment of that line. The line dictionary has the endpoints of
    the line segment (keys x_1, y_1, x_2, and y_2)."""
    x, y = point
    if (min(line['x_1'], line['x_2']) <= x <= max(line['x_1'], line['x_2']) and
        min(line['y_1'], line['y_2']) <= y <= max(line['y_1'], line['y_2'])):
        return True
    else:
        return False

def dist_between_points(point_1, point_2):
    """Returns the absolute distance between two points."""
    x_1, y_1 = point_1
    x_2, y_2 = point_2
    return sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)

def dist_point_to_line(line, point):
    """Returns the minimum distance between a point and a line."""
    x, y = point
    return ( (line['a'] * x + line['b'] * y - line['c']) / 
            sqrt(line['a']**2 + line['b']**2))

def get_line_dict(x_1, y_1, x_2, y_2):
    """Returns a dictionary with the line's endpoints, as well as its linear
    coefficients ax + by = c."""
    line_dict = {'x_1': x_1, 
                 'y_1': y_1,
                 'x_2': x_2,
                 'y_2': y_2,
                 'a': y_2 - y_1,
                 'b': x_1 - x_2,
                 'c': (y_2 - y_1) * x_1 + (x_1 - x_2) * y_1}
    return line_dict


class Laser(object):
    def __init__(self, pose):
        self.pose = pose
        self.min_angle = d.LASER_MIN_ANGLE
        self.max_angle = d.LASER_MAX_ANGLE
        self.resolution = d.LASER_RES
        self.range = d.LASER_RANGE
        self.noise = d.LASER_NOISE
        self.freq = d.LASER_FREQ

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
            laser_beams.append(get_line_dict(x, y, x_2, y_2))
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
                if dist_point_to_line(line, (self.pose[0], self.pose[1])) > self.range:
                    continue
                # find the (x, y) coordinates of intersection (if it exists)
                intersection = find_intersection(beam, line)
                # if the lines intersect
                if intersection and validate_intersection(line, intersection):
                    # if the intersection is on both the line segments
                    if (validate_intersection(line, intersection) and 
                            validate_intersection(beam, intersection)):
                        # get a range measurement
                        r_temp = dist_between_points((self.pose[0], self.pose[1]), intersection)
                        # keep it if its in the max range and its the smallest seen yet
                        if r_temp < r_min and r_temp < self.range:
                            r_min = r_temp + random.gauss(0, self.noise)
            r_min = r_min if r_min < 999 else 0
            ranges.append(r_min)
        return ranges


class Odometer(object):
    def __init__(self):
        self.resolution = d.ODOM_RES
        self.noise = d.ODOM_NOISE
        self.freq = d.ODOM_FREQ


class Robot(object):
    """Contains all the data and methods pertaining to the robot and its
    sensor(s)."""
    def __init__(self):
        self.x, self.y, self.heading = d.ROBOT_INIT_POSE
        self.width = d.ROBOT_WIDTH
        self.length = d.ROBOT_LENGTH
        self.wheelbase = d.ROBOT_WHEELBASE
        self.wheel_rad = d.ROBOT_WHEEL_RAD
        self.max_vel = d.ROBOT_MAX_VEL
        self.max_ang_vel = d.ROBOT_MAX_ANG_VEL * pi/180
        self.vel = 0.0
        self.ang_vel = 0.0
        self.moved = False # flag to determine if robot should be moved
        self.scanned = False # flag to determine if laser should be redrawn
        self.sized = False # flag to determine if robot should be redrawn
        self.last_scan = None # (pose, ranges) of latest laser scan
        self.laser = Laser(self.pose)
        self.odometer = Odometer()

    def translate(self, distance):
        """Update the (x, y) position of the robot after moving it forward a set
        distance."""
        self.x = self.x + cos(self.heading) * distance
        self.y = self.y + sin(self.heading) * distance

    @property
    def pose(self):
        return (self.x, self.y, self.heading)

    def rotate(self, angle):
        """Update the heading of the robot after rotating it a set angle."""
        self.heading = self.heading + angle

    def update_pose(self):
        """Update the pose of the robot based on its velocity and odometry
        update rate."""
        if abs(self.vel) < 1e-5:
            self.vel = 0
        else:
            self.translate(self.vel * 1.0/d.ODOM_FREQ)
            self.moved = True
        if abs(self.ang_vel) < 1e-5:
            self.ang_vel = 0
        else:
            self.rotate(self.ang_vel * 1.0/d.ODOM_FREQ)
            self.moved = True

    def scan_laser(self, line_map):
        """Scan the laser and append the resulting ranges and the current pose 
        to the scan history."""
        # update laser pose to match robot pose
        self.laser.pose = self.pose
        # scan laser and append the current pose and ranges to the scan histroy
        ranges = self.laser.scan(line_map)
        self.last_scan = (self.pose, ranges)
        self.scanned = True

    def set_width(self, width):
        self.width = width
        self.sized = True

    def set_length(self, length):
        self.length = length
        self.sized = True

    def set_wheelbase(self, wheelbase):
        self.wheelbase = wheelbase

    def set_wheel_rad(self, wheel_rad):
        self.wheel_rad = wheel_rad
