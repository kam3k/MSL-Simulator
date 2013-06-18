# Python imports
import random
from numpy import linspace
from math import sin, cos, pi, sqrt, floor, atan2, atan, degrees, radians

# MSL Sim imports
import sim.defaults as d


def dist_between_points(point_1, point_2):
    """Returns the absolute distance between two points."""
    x_1, y_1 = point_1
    x_2, y_2 = point_2
    return sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)

def dist_point_to_line(line, point):
    """Returns the minimum distance between a point and a line."""
    x, y = point
    return ( abs(line['a'] * x + line['b'] * y - line['c']) / 
            sqrt(line['a']**2 + line['b']**2))

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

def get_line_dict(x_1, y_1, x_2, y_2):
    """Returns a dictionary with the line's endpoints, as well as its linear
    coefficients ax + by = c."""
    line_dict = {'p_1': (x_1, y_1),
                 'p_2': (x_2, y_2),
                 'x_1': x_1, 
                 'y_1': y_1,
                 'x_2': x_2,
                 'y_2': y_2,
                 'a': y_2 - y_1,
                 'b': x_1 - x_2,
                 'c': (y_2 - y_1) * x_1 + (x_1 - x_2) * y_1}
    return line_dict

def pi_to_pi(angle, deg=False):
    if deg:
        angle = radians(angle)
    angle = angle % (2*pi)
    if angle > pi:
        angle -= 2*pi
    angle = angle if not deg else degrees(angle)
    return angle

def validate_intersection(line, point):
    """Determines whether or not a point that is known to be on a line is on a 
    particular segment of that line. The line dictionary has the endpoints of
    the line segment (keys x_1, y_1, x_2, and y_2)."""
    eps = 1e-5 # floating point precision tie breaker (vert and horiz lines)
    x, y = point
    if (min(line['x_1'], line['x_2']) - eps <= x <= max(line['x_1'], line['x_2']) + eps and
        min(line['y_1'], line['y_2']) - eps <= y <= max(line['y_1'], line['y_2']) + eps):
        return True
    else:
        return False


class Laser(object):
    def __init__(self, pose):
        self.pose = pose
        self.min_angle = d.LASER_MIN_ANGLE
        self.max_angle = d.LASER_MAX_ANGLE
        self.resolution = d.LASER_RES
        self.range = d.LASER_RANGE
        self.noise = d.LASER_NOISE
        self.freq = d.LASER_FREQ

    def __get_laser_beams(self):
        """Given the pose of the robot, returns a list of line dictionaries. 
        Each dictionary contains information about a single beam in the laser 
        scan."""
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
        laser_beams = self.__get_laser_beams()
        position = (self.pose[0], self.pose[1])
        lines_in_range = []
        # Only include lines inside the laser range
        for line in line_map:
            perp_dist = dist_point_to_line(line, position)
            if perp_dist > self.range:
                continue
            else:
                close_point = None
                # Find point on full line where perpindicular intersects
                x_1, y_1 = line['p_1']
                x_2, y_2 = line['p_2']
                x_3, y_3 = position
                k = ((y_2-y_1) * (x_3-x_1) - (x_2-x_1) * (y_3-y_1)) / (
                        (y_2-y_1)**2 + (x_2-x_1)**2)
                x_4 = x_3 - k * (y_2-y_1)
                y_4 = y_3 + k * (x_2-x_1)
                # If this point is on the line, add it to the list
                if validate_intersection(line, (x_4, y_4)):
                    close_point = (x_4, y_4)
                else:
                    # Add the line if one of the endpoints is in range
                    if dist_between_points(line['p_1'], position) < self.range:
                        close_point = line['p_1']
                    elif dist_between_points(line['p_2'], position) < self.range:
                        close_point = line['p_2']
                if close_point:
                    bearing = atan2(close_point[1] - position[1],
                                    close_point[0] - position[0])
                    bearing = degrees(pi_to_pi(bearing))
                    heading = degrees(self.pose[2])
                    min_angle = pi_to_pi(self.min_angle + heading, deg=True)
                    max_angle = pi_to_pi(self.max_angle + heading, deg=True)
                    if min_angle < max_angle:
                        if min_angle < bearing < max_angle:
                            lines_in_range.append(line)
                    else:
                        if bearing > min_angle or bearing < max_angle:
                            lines_in_range.append(line)
                    #lines_in_range.append(line)

        print len(line_map), len(lines_in_range)
        for beam in laser_beams:
            r_min = 999
            for line in lines_in_range:
                # find the (x, y) coordinates of intersection (if it exists)
                intersection = find_intersection(beam, line)
                # if the lines intersect
                if intersection and validate_intersection(line, intersection):
                    # if the intersection is on both the line segments
                    if (validate_intersection(line, intersection) and 
                            validate_intersection(beam, intersection)):
                        # get a range measurement
                        r_temp = dist_between_points(position, intersection)
                        # keep it if its in the max range and its the 
                        # smallest seen yet
                        if r_temp < r_min and r_temp < self.range:
                            r_min = r_temp + random.gauss(0, self.noise)
            r_min = r_min if r_min < 999 else 0
            ranges.append(r_min)
        return ranges


class Odometer(object):
    """A simple odometer with resolution, frequency and noise properties."""
    def __init__(self):
        self.res = d.ODOM_RES
        self.freq = d.ODOM_FREQ
        self.noise = d.ODOM_NOISE

    def read(self, vel, ang_vel, wheel_rad, wheelbase):
        """Returns a tuple (right_angle, left_angle) that indicates the angle
        the odometers have turned since in one period."""
        # return zero if not moving
        if vel == 0 and ang_vel == 0:
            return (0.0, 0.0)
        # Get angular velocities of each side
        omega_r = vel + wheelbase/(2*wheel_rad) * ang_vel
        omega_l = vel - wheelbase/(2*wheel_rad) * ang_vel
        # Calculate change of angle in this time step
        theta_r = omega_r * (1.0/self.freq)
        theta_l = omega_l * (1.0/self.freq)
        # Calculate number of ticks for this change
        ticks_r = theta_r / (self.res * pi/180) + random.gauss(0, self.noise)
        ticks_l = theta_l / (self.res * pi/180) + random.gauss(0, self.noise)
        ticks_r = int(floor(ticks_r))
        ticks_l = int(floor(ticks_l))
        # Convert back to angle (in radians this time)
        noisy_theta_r = ticks_r * self.res * pi/180 
        noisy_theta_l = ticks_l * self.res * pi/180
        return (noisy_theta_r, noisy_theta_l)


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
        self.scanned = False # flag to determine if laser should be redrawn
        self.changed = False # flag to determine if robot should be redrawn
        self.last_scan = None # (pose, ranges) of latest laser scan
        self.last_odom = None # number of ticks of latest odometry measurement
        self.laser = Laser(self.pose)
        self.odometer = Odometer()

    def __translate(self, distance):
        """Update the (x, y) position of the robot after moving it forward a set
        distance."""
        self.x = self.x + cos(self.heading) * distance
        self.y = self.y + sin(self.heading) * distance

    @property
    def pose(self):
        """Returns a tuple of the pose (x, y, heading) in [m, m, rad]."""
        return (self.x, self.y, self.heading)

    def __rotate(self, angle):
        """Update the heading of the robot after rotating it a set angle."""
        self.heading += angle
        # make sure heading is between -pi and pi
        self.heading = pi_to_pi(self.heading)

    def update_pose(self):
        """Update the pose of the robot based on its velocity and odometry
        update rate."""
        if abs(self.vel) < 1e-5:
            self.vel = 0
        else:
            self.__translate(self.vel * 1.0/self.odometer.freq)
            self.changed = True
        if abs(self.ang_vel) < 1e-5:
            self.ang_vel = 0
        else:
            self.__rotate(self.ang_vel * 1.0/self.odometer.freq)
            self.changed = True
        # Return odometry measurement
        return self.odometer.read(self.vel, self.ang_vel, self.wheel_rad,
                self.wheelbase)

    def scan_laser(self, line_map):
        """Scan the laser and append the resulting ranges and the current pose 
        to the scan history."""
        # update laser pose to match robot pose
        self.laser.pose = self.pose
        # scan laser and save it with the robot pose
        self.scanned = True
        return self.laser.scan(line_map)
        return self.width

    def set_width(self, width):
        """Sets the width of the robot and activates the 'changed' flag
        indicating that a property of the robot has changed."""
        self.width = width
        self.changed = True

    def set_length(self, length):
        """Sets the length of the robot and activates the 'changed' flag
        indicating that a property of the robot has changed."""
        self.length = length
        self.changed = True
