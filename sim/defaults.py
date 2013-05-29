# plot/map
MAP_WIDTH = 10 # [m]
MAP_HEIGHT = 10 # [m]
PLOT_FREQ = 10 # how often the plot is refreshed [Hz]

# robot
ROBOT_INIT_POSE = (0, 0, 0) # (x, y, heading) [m, m, rad]
ROBOT_WIDTH = 0.5 # [m]
ROBOT_LENGTH = 0.8 # [m]
ROBOT_WHEELBASE = 0.5 # [m]
ROBOT_WHEEL_RAD = 0.2 # [m]
ROBOT_MAX_VEL = 0.5 # [m/s]
ROBOT_MAX_ANG_VEL = 60 # [deg/s]

# laser
LASER_MIN_ANGLE = -90 # minimum angle [deg]
LASER_MAX_ANGLE = 90 # maximum angle [deg]
LASER_RES = 0.5 # angular resolution [deg]
LASER_RANGE = 5.0 # range [m]
LASER_NOISE = 0.01 # standard deviation on range measurement [m]
LASER_FREQ = 10 # how often the laser is scanned [Hz]

# odometry
ODOM_RES = 16 # [bits]
ODOM_FREQ = 10 # how often the odometry is update [Hz]
ODOM_NOISE = 0.001 # [rad]
