# ------------------------------------------------------------------------------
# LASERS
# ------------------------------------------------------------------------------

# Default custom
LASER_MIN_ANGLE = -90 # minimum angle [deg]
LASER_MAX_ANGLE = 90 # maximum angle [deg]
LASER_RES = 0.5 # angular resolution [deg]
LASER_RANGE = 5.0 # range [m]
LASER_NOISE = 0.01 # standard deviation on range measurement [m]
LASER_FREQ = 10 # how often the laser is scanned [Hz]

# SICK LMS111
SICK_111_MIN_ANGLE = -135 # [deg]
SICK_111_MAX_ANGLE = 135 # [deg]
SICK_111_RES = 0.25 # [deg]
SICK_111_RANGE = 20 # [m]
SICK_111_NOISE = 0.02 # [m]
SICK_111_FREQ = 5 # [Hz] # Actually 25, smaller to reduce slowdown for now

# Hokuyo URG-04LX
HOK_04_MIN_ANGLE = -120 # [deg]
HOK_04_MAX_ANGLE = 120 # [deg]
HOK_04_RES = 0.36 # [deg]
HOK_04_RANGE = 4.095 # [m]
HOK_04_NOISE = 0.02 # [m] (in reality, 1% of measurement, 2 cm is half range)
HOK_04_FREQ = 10 # [Hz]

# ------------------------------------------------------------------------------
# ODOMETERS
# ------------------------------------------------------------------------------

# Default custom
ODOM_RES = 0.5 # [deg]
ODOM_FREQ = 5 # how often the odometry is updated [Hz]
ODOM_NOISE = 0.5 # ticks

# ------------------------------------------------------------------------------
# ROBOTS
# ------------------------------------------------------------------------------

# Default custom
ROBOT_INIT_POSE = (0, 0, 0) # (x, y, heading) [m, m, rad]
ROBOT_WIDTH = 0.5 # [m]
ROBOT_LENGTH = 0.8 # [m]
ROBOT_WHEELBASE = 0.5 # [m]
ROBOT_WHEEL_RAD = 0.2 # [m]
ROBOT_MAX_VEL = 0.5 # [m/s]
ROBOT_MAX_ANG_VEL = 30 # [deg/s]

# Clearpath Husky A200
HUSKY_WIDTH = 0.670 # [m]
HUSKY_LENGTH = 0.990 # [m]
HUSKY_WHEELBASE = 0.545 # [m]
HUSKY_WHEEL_RAD = 0.165 # [m]
HUSKY_MAX_VEL = 1.0 # [m/s]
HUSKY_MAX_ANG_VEL = 154 # [deg/s]

# MobileRobots P3AT
P3AT_WIDTH = 0.497 # [m]
P3AT_LENGTH = 0.508 # [m]
P3AT_WHEELBASE = 0.381 # [m]
P3AT_WHEEL_RAD = 0.111 # [m]
P3AT_MAX_VEL = 0.7 # [m/s]
P3AT_MAX_ANG_VEL = 148 # [deg/s]

# ------------------------------------------------------------------------------
# MISCELLANEOUS
# ------------------------------------------------------------------------------

# plot/map
MAP_WIDTH = 10 # [m]
MAP_HEIGHT = 10 # [m]
PLOT_FREQ = 10 # how often the plot is refreshed [Hz]

# other
VELOCITY_INCREMENT = 0.1 # amount the velocity changes per key press [m/s]
ANG_VELOCITY_INCREMENT = 5 # amount the ang. velocity changes per key press [deg/s]

