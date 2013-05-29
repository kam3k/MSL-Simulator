# Python imports
import math

# MSL Sim imports
import sim.defaults as d
import sim.model.laser as laser

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
        self.max_ang_vel = d.ROBOT_MAX_ANG_VEL
        self.vel = 0
        self.ang_vel = 0
        self.moved = False # flag to determine if robot should be moved
        self.scanned = False # flag to determine if laser should be redrawn
        self.sized = False # flag to determine if robot should be redrawn
        self.scan_history = [] # (pose, ranges) of every laser scan
        self.laser = laser.Laser(self.pose)

    def translate(self, distance):
        """Update the (x, y) position of the robot after moving it forward a set
        distance."""
        self.x = self.x + math.cos(self.heading) * distance
        self.y = self.y + math.sin(self.heading) * distance

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
        self.scan_history.append((self.pose, ranges))
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
