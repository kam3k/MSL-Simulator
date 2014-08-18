# Python imports
import random
from numpy import linspace
from math import sin, cos, pi, sqrt, floor, atan2, degrees, radians

# MSL Sim imports
import sim.defaults as d

def circle_intersections(line, circle_centre, circle_rad):
    # Adjust coordinates so circle is at (0,0)
    x_1 = line['x_1'] - circle_centre[0]
    y_1 = line['y_1'] - circle_centre[1]
    x_2 = line['x_2'] - circle_centre[0]
    y_2 = line['y_2'] - circle_centre[1]
    r = circle_rad
    # Intermediate variables
    d_x = x_2 - x_1
    d_y = y_2 - y_1
    d_r = sqrt(d_x**2 + d_y**2)
    D = x_1*y_2 - x_2*y_1
    sgn = -1 if d_y < 0 else 1
    disc = r**2 * d_r**2 - D**2
    # Return if the discriminant is not greater than zero (ignoring tangent
    # case)
    if disc <= 0:
        return None
    # Calculate the two intersections p_1 and p_2
    p_1_x = (D * d_y + sgn * d_x * sqrt(r**2 * d_r**2 - D**2)) / d_r**2
    p_1_y = (-D * d_x + abs(d_y) * sqrt(r**2 * d_r**2 - D**2)) / d_r**2
    p_2_x = (D * d_y - sgn * d_x * sqrt(r**2 * d_r**2 - D**2)) / d_r**2
    p_2_y = (-D * d_x - abs(d_y) * sqrt(r**2 * d_r**2 - D**2)) / d_r**2
    # Adjust coordinates so circle is back to its original coordinates
    p_1_x += circle_centre[0]
    p_1_y += circle_centre[1]
    p_2_x += circle_centre[0]
    p_2_y += circle_centre[1]
    return ((p_1_x, p_1_y), (p_2_x, p_2_y))

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

    def __in_range(self, point):
        """Determines whether or not a point is within the range of the laser."""
        position = (self.pose[0], self.pose[1])
        return dist_between_points(point, position) < self.range
        
    def __in_FOV(self, point):
        """Determines whether or not a point is within the FOV of the laser."""
        position = (self.pose[0], self.pose[1])
        bearing = atan2(point[1] - position[1],
                        point[0] - position[0])
        bearing = degrees(pi_to_pi(bearing))
        heading = degrees(self.pose[2])
        min_angle = pi_to_pi(self.min_angle + heading, deg=True)
        max_angle = pi_to_pi(self.max_angle + heading, deg=True)
        if min_angle < max_angle:
            if min_angle < bearing < max_angle:
                return True
        elif bearing > min_angle or bearing < max_angle:
            return True
        return False

    def __include_line(self, line, min_line, max_line):
        position = (self.pose[0], self.pose[1])
        # Eliminate far, short lines
        length = dist_between_points(line['p_1'], line['p_2'])
        dist_to_p1 = dist_between_points(position, line['p_1'])
        dist_to_p2 = dist_between_points(position, line['p_2'])
        if dist_to_p1 > self.range + length and dist_to_p2 > self.range + length:
            return False
        # Keep lines who have an endpoint in range and FOV
        if self.__in_range(line['p_1']) and self.__in_FOV(line['p_1']):
            return True
        if self.__in_range(line['p_2']) and self.__in_FOV(line['p_2']):
            return True
        # Keep lines that intersect edge of laser FOV and are on both lines
        intersect_min = find_intersection(line, min_line)
        if (validate_intersection(line, intersect_min) and 
                validate_intersection(min_line, intersect_min)):
            return True
        intersect_max = find_intersection(line, max_line)
        if (validate_intersection(line, intersect_max) and 
                validate_intersection(max_line, intersect_max)):
            return True
        # Keep lines that intersect the range circle within the FOV
        intersections = circle_intersections(line, position, self.range)
        if intersections is None:
            return False
        inter_1, inter_2 = intersections
        if validate_intersection(line, inter_1) and self.__in_FOV(inter_1):
            return True
        if validate_intersection(line, inter_2) and self.__in_FOV(inter_2):
            return True
        return False
    
    def __reduce_line_map(self, line_map):
        # Get lines at edges of FOV of laser
        x, y, theta = self.pose
        x_min = x + self.range * cos(theta + pi/180*self.min_angle)
        y_min = y + self.range * sin(theta + pi/180*self.min_angle)
        x_max = x + self.range * cos(theta + pi/180*self.max_angle)
        y_max = y + self.range * sin(theta + pi/180*self.max_angle)
        min_line = get_line_dict(x, y, x_min, y_min)
        max_line = get_line_dict(x, y, x_max, y_max)
        # Filter out lines
        kept_lines = []
        for line in line_map:
            if self.__include_line(line, min_line, max_line):
                kept_lines.append(line)
        return kept_lines

    def scan(self, line_map):
        """Given the pose of the robot and a list of line segments (line_map),
        returns a list of range measurements."""
        ranges = []
        laser_beams = self.__get_laser_beams()
        position = (self.pose[0], self.pose[1])
        # Only include lines inside the laser range
        kept_lines = self.__reduce_line_map(line_map)
        for beam in laser_beams:
            r_min = 999
            for line in kept_lines:
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
        self.right_partial_tick = 0.0 # fraction of tick left over from [0-1]
        self.left_partial_tick = 0.0

    def read(self, vel, ang_vel, wheel_rad, wheelbase):
        """Returns a tuple (ticks_right, ticks_left) that indicates the number
        of ticks the odometers have turned in one period."""
        # return None if not moving
        if vel == 0 and ang_vel == 0:
            return None
        # Get angular velocities of each side
        omega_r = vel + wheelbase/(2*wheel_rad) * ang_vel
        omega_l = vel - wheelbase/(2*wheel_rad) * ang_vel
        # Calculate change of angle in this time step
        theta_r = omega_r * (1.0/self.freq)
        theta_l = omega_l * (1.0/self.freq)
        # Calculate (float) number of ticks for this change
        ticks_r = theta_r / (self.res * pi/180) + random.gauss(0, self.noise)
        ticks_l = theta_l / (self.res * pi/180) + random.gauss(0, self.noise)
        # Add the partial tick from last time
        ticks_r += self.right_partial_tick
        ticks_l += self.left_partial_tick
        # Floor and convert to integer
        ticks_r_int = int(floor(ticks_r))
        ticks_l_int = int(floor(ticks_l))
        # Calculate new left over ticks
        self.right_partial_tick = ticks_r - ticks_r_int
        self.left_partial_tick = ticks_l - ticks_l_int
        return ticks_r_int, ticks_l_int


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
