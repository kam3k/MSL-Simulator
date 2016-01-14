# MSL Simulator
MSL Simulator is a two-dimensional mobile robot simulator designed to be used with the [Robot Operating System (ROS)](http://www.ros.org). Created by [Marc Gallant](http://marcgallant.ca), originally for use in the [Mining Systems Laboratory](http://msl.engineering.queensu.ca). Here is what the MSL Simulator looks like in action:

![MSL Simulator](images/msl_sim.gif)

## Installation

## Basic Usage
The robot (blue rectangle) uses its laser scanner (red semi-circle) as it drives through a randomly generated field of obstacles (squares). The main view window on the right is a static top down view of the environment, and the secondary view window on the left is a GPS-like view of the robot coordinate frame. The information panel at the top left displays basic information about the robot and contains buttons to change the zoom level, load a map, or change settings.

### Launching the Simulator
The simulator is launched much like any other ROS node. There must be a `roscore` running, and then to launch the simulator execute the command

```
rosrun msl_sim main.py
```

### Controlling the Robot
Click on either of the two view windows to give it focus. The following keyboard commands move the robot:

*w*: increase the linear velocity by a single step

*s*: decrease the linear velocity by a single step

*a*: increase the angular velocity by a single step (turn left)

*d*: decrease the angular velocity by a single step (turn right)

*spacebar*: stop the robot (set both linear and angular velocities to zero).

The velocity step sizes can be adjusted in the "General" tab of the settings dialog.

### Sensors
The robot is equipped with five sensors: a two-dimensional laser scanner, wheel encoders, a gyroscope, a compass, and GPS. These sensors can be configured by clicking on the "Settings..." button, which brings up this dialog window:

![Settings](images/settings_dialog.png)

From here you can edit individual settings of the robot and each sensor, or (in some cases) you can pick preset values from real hardware. For example, you can choose the Hokuyo URG-04LX as your laser scanner and it will automatically choose all the right settings:

![Hokuyo](images/hokuyo.png)

Note that all changes to the sensor settings occur in *real time*. For example, watch what happens while I adjust the range of the laser scanner:

![Laser](images/laser.gif)

## Generating Maps
A map is a text file listing line segments. Here is a simple example of a map file:

```
# Lines beginning with '#' are ignored
# x1 y1 x2 y2
1.32 -8.32 2.41 -11.33
5.11 -4.11 -1.12 -3.45
```

This map consists of two line segments. The first is a line from coordinates (1.32, -8.32) to (2.41, -11.33), and the second is a line segment from coordinates (5.11, -4.11) to (-1.12, -3.45).

Included in the `maps` directory are two python scripts to autogenerate map files. The map in the above images was generated using `random_landmarks_generator.py`. Near the top of the script are some parameters that can be configured to generate any number of landmarks of various sizes. The file `random_wall_generator.py` generates a random length of jagged wall based on a number of parameters (configurable near the top of the file).

Running either of these scripts is as simple as

```
python name_of_script.py name_of_generated_map_file.txt
```

## Recording Data
The simulator publishes six ros topics:

- `/msl_sim/compass` has message type msl_sim/Compass, which is the timestamped bearing measured by the compass.
- `/msl_sim/encoders` has message type msl_sim/Encoders, which is the timestamped number of left and right ticks recorded by the encoders since the last timestamp. This topic only publishes data why the robot is in motion.
- `/msl_sim/gps` has message type msl_sim/GPS, which is the timestamped GPS measurement of the (x, y) position of the robot.
- `/msl_sim/ground_truth` has message type msl_sim/Pose2DStamped, which is the timestamped true pose of the robot.
- `/msl_sim/gyro` has message type msl_sim/Gyro, which is the timestamped angular velocity of the robot measured by the gyroscope.
- `/msl_sim/scan` has message type sensor_msgs/LaserScan, which is the timestamped bearing/ranges of the data collected by the laser scanner.

Recording the data is done using the usual ROS tools (i.e., [rosbag](http://wiki.ros.org/rosbag/Commandline)). For example, to record just the laser scan, the command is

```
rosbag record /msl_sim/scan -O name_of_output_file
```