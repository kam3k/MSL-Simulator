# Python imports
import math
import time
import random
import os

# PySide imports
from PySide import QtGui, QtCore

# ROS imports
import rospy
import rospkg
from sensor_msgs.msg import LaserScan

# MSL Sim imports
import sim.model as mod
import sim.defaults as d
from msl_sim.msg import Compass, Gyro, Encoders, SimInfo


class MainWindow(QtGui.QMainWindow):
    """The main window. This window displays all the widgets."""
    def __init__(self):
        super(MainWindow, self).__init__()
        self.robot = mod.Robot()
        self.loadGUI()
        # ROS
        self.sim_info_publisher = rospy.Publisher('/msl_sim/sim_info', SimInfo, queue_size=10)
        # Place and scale the logo
        pkg_dir = rospkg.RosPack().get_path('msl_sim')
        print os.path.join(pkg_dir,'images', 'msl_logo.png')
        pixmap = QtGui.QPixmap(os.path.join(pkg_dir, 'src', 'img', 'msl_logo.png'))
        self.main.logo_label.setPixmap(pixmap)
        self.main.logo_label.setAlignment(QtCore.Qt.AlignCenter)
        # Set initial scale of main plot
        self.main.graphics_view.set_scale(0.6)
        # Give the zoomed in plotting area the same scene as the zoomed out one
        self.main.graphics_view_zoom.setScene(self.main.graphics_view.scene())
        # Initialize the camera in the zoomed in plotting window
        self.main.graphics_view_zoom.initializeCamera(self.robot)
        # Give the zoomed-out plotting area a copy of the zoomed-in plotting
        # area so it can change it based on its timers
        self.main.graphics_view.zoom = self.main.graphics_view_zoom
        # Give the plotting area the same robot as the rest of the GUI and start
        # updating it via timers
        self.main.graphics_view.robot = self.robot
        self.main.graphics_view.initialiseRobot()
        self.settings_to_default()
        # Start a timer that updates the labels
        self.label_timer = QtCore.QTimer()
        self.label_timer.timeout.connect(self.update_info_labels)
        self.label_timer.start()
        # Start timers that update the plot and the model
        self.main.graphics_view.start_timers()
        # Publish the initial simulation information in ROS
        self.publish_sim_info_msg()

    # --------------------------------------------------------------------------
    # SETUP METHODS
    # --------------------------------------------------------------------------
    def connect_signals_to_slots(self):
        # -----
        # ROBOT
        # -----
        self.settings.robot_ang_vel_box.valueChanged.connect(
                self.robot_ang_vel_changed)
        self.settings.robot_ang_vel_slider.valueChanged.connect(
                self.robot_ang_vel_changed)
        self.settings.robot_combo.currentIndexChanged.connect(
                self.robot_combo_changed)
        self.settings.robot_length_box.valueChanged.connect(
                self.robot_length_changed)
        self.settings.robot_length_slider.valueChanged.connect(
                lambda: self.robot_length_changed(
                    self.settings.robot_length_slider.value()/10.0))
        self.settings.robot_wheel_rad_box.valueChanged.connect(
                self.robot_wheel_rad_changed)
        self.settings.robot_wheel_rad_slider.valueChanged.connect(
                lambda: self.robot_wheel_rad_changed(
                    self.settings.robot_wheel_rad_slider.value()/100.0))
        self.settings.robot_wheelbase_box.valueChanged.connect(
                self.robot_wheelbase_changed)
        self.settings.robot_wheelbase_slider.valueChanged.connect(
                lambda: self.robot_wheelbase_changed(
                    self.settings.robot_wheelbase_slider.value()/10.0))
        self.settings.robot_width_box.valueChanged.connect(
                self.robot_width_changed)
        self.settings.robot_width_slider.valueChanged.connect(
                lambda: self.robot_width_changed(
                    self.settings.robot_width_slider.value()/10.0))
        self.settings.robot_vel_box.valueChanged.connect(self.robot_vel_changed)
        self.settings.robot_vel_slider.valueChanged.connect(
                lambda: self.robot_vel_changed(
                    self.settings.robot_vel_slider.value()/10.0))

        # --------
        # ODOMETER
        # --------
        self.settings.odom_freq_box.valueChanged.connect(self.odom_freq_changed)
        self.settings.odom_freq_slider.valueChanged.connect(self.odom_freq_changed)
        self.settings.odom_noise_box.valueChanged.connect(self.odom_noise_changed)
        self.settings.odom_noise_slider.valueChanged.connect(
                lambda: self.odom_noise_changed(
                    self.settings.odom_noise_slider.value()/10.0))
        self.settings.odom_res_box.valueChanged.connect(self.odom_res_changed)
        self.settings.odom_res_slider.valueChanged.connect(
                lambda: self.odom_res_changed(
                    self.settings.odom_res_slider.value()/100.0))

        # -----
        # LASER
        # -----
        self.settings.laser_combo.currentIndexChanged.connect(self.laser_combo_changed)
        self.settings.laser_freq_box.valueChanged.connect(
                self.laser_freq_changed)
        self.settings.laser_freq_slider.valueChanged.connect(
                self.laser_freq_changed)
        self.settings.laser_max_bear_box.valueChanged.connect(
                self.laser_max_bear_changed)
        self.settings.laser_max_bear_slider.valueChanged.connect(
                self.laser_max_bear_changed)
        self.settings.laser_min_bear_box.valueChanged.connect(
                self.laser_min_bear_changed)
        self.settings.laser_min_bear_slider.valueChanged.connect(
                self.laser_min_bear_changed)
        self.settings.laser_noise_box.valueChanged.connect(
                self.laser_noise_changed)
        self.settings.laser_noise_slider.valueChanged.connect(
                self.laser_noise_changed)
        self.settings.laser_range_box.valueChanged.connect(
                self.laser_range_changed)
        self.settings.laser_range_slider.valueChanged.connect(
                self.laser_range_changed)
        self.settings.laser_res_box.valueChanged.connect(
                self.laser_res_changed)
        self.settings.laser_res_slider.valueChanged.connect(
                lambda: self.laser_res_changed(
                    self.settings.laser_res_slider.value()/10.0))

        # -----
        # MAPPING
        # -----
        self.main.load_map_button.clicked.connect(self.load_map)

        # -----
        # SETTINGS
        # -----
        self.main.settings_button.clicked.connect(self.open_settings_dialog)

        # -----
        # OTHER
        # -----
        self.settings.ang_vel_inc_box.valueChanged.connect(self.ang_vel_inc_changed)
        self.settings.laser_check.stateChanged.connect(self.laser_check_changed)
        self.settings.map_check.stateChanged.connect(self.map_check_changed)
        self.settings.vel_inc_box.valueChanged.connect(self.vel_inc_changed)
        self.main.zoom_in_button.clicked.connect(self.zoom_in)
        self.main.zoom_out_button.clicked.connect(self.zoom_out)


    def loadGUI(self):
        """Load the GUI from the .py file that was generated by the .ui file
        using the pyside-uic tool."""
        # import generated .py file here to prevent circular reference
        from sim.main_window import Ui_main_window
        self.main = Ui_main_window()
        self.main.setupUi(self)
        from sim.settings_dialog import Ui_settings_dialog
        self.settings = Ui_settings_dialog()
        self.dialog = QtGui.QDialog()
        self.settings.setupUi(self.dialog)
        self.connect_signals_to_slots()
        # Add items to pull down menus
        self.settings.laser_combo.addItems(["Custom", "SICK LMS111",
            "Hokuyo URG-04LX"])
        self.settings.robot_combo.addItems(["Custom", "Clearpath Husky A200",
            "MobileRobots P3AT"])

    # --------------------------------------------------------------------------
    # SLOTS
    # --------------------------------------------------------------------------

    # -----
    # ROBOT
    # -----
    def robot_ang_vel_changed(self, value):
        self.settings.robot_ang_vel_slider.setValue(value)
        self.settings.robot_ang_vel_box.setValue(value)
        self.robot.max_ang_vel = value * math.pi/180

    def robot_combo_changed(self, value):
        if value == 0: # Custom
            self.toggle_enable_robot_settings(True)
        elif value == 1: # Clearpath Husky A200
            self.robot_length_changed(d.HUSKY_LENGTH)
            self.robot_width_changed(d.HUSKY_WIDTH)
            self.robot_wheel_rad_changed(d.HUSKY_WHEEL_RAD)
            self.robot_wheelbase_changed(d.HUSKY_WHEELBASE)
            self.robot_vel_changed(d.HUSKY_MAX_VEL)
            self.robot_ang_vel_changed(d.HUSKY_MAX_ANG_VEL)
            self.toggle_enable_robot_settings(False)
        elif value == 2: # MobileRobots P3AT
            self.robot_length_changed(d.P3AT_LENGTH)
            self.robot_width_changed(d.P3AT_WIDTH)
            self.robot_wheel_rad_changed(d.P3AT_WHEEL_RAD)
            self.robot_wheelbase_changed(d.P3AT_WHEELBASE)
            self.robot_vel_changed(d.P3AT_MAX_VEL)
            self.robot_ang_vel_changed(d.P3AT_MAX_ANG_VEL)
            self.toggle_enable_robot_settings(False)

    def robot_length_changed(self, value):
        self.settings.robot_length_slider.setValue(int(10*value))
        self.settings.robot_length_box.setValue(value)
        self.robot.set_length(value)

    def robot_wheel_rad_changed(self, value):
        self.settings.robot_wheel_rad_slider.setValue(int(100*value))
        self.settings.robot_wheel_rad_box.setValue(value)
        self.robot.wheel_rad = value
        self.publish_sim_info_msg()

    def robot_wheelbase_changed(self, value):
        self.settings.robot_wheelbase_slider.setValue(int(10*value))
        self.settings.robot_wheelbase_box.setValue(value)
        self.robot.wheelbase = value
        self.publish_sim_info_msg()

    def robot_width_changed(self, value):
        self.settings.robot_width_slider.setValue(int(10*value))
        self.settings.robot_width_box.setValue(value)
        self.robot.set_width(value)

    def robot_vel_changed(self, value):
        self.settings.robot_vel_slider.setValue(int(10*value))
        self.settings.robot_vel_box.setValue(value)
        self.robot.max_vel = value

    # --------
    # ODOMETER
    # --------
    def odom_freq_changed(self, value):
        self.settings.odom_freq_slider.setValue(value)
        self.settings.odom_freq_box.setValue(value)
        self.robot.odometer.freq = value

    def odom_noise_changed(self, value):
        self.settings.odom_noise_slider.setValue(int(10*value))
        self.settings.odom_noise_box.setValue(value)
        self.robot.odometer.noise = value
        self.publish_sim_info_msg()

    def odom_res_changed(self, value):
        self.settings.odom_res_slider.blockSignals(True)
        self.settings.odom_res_slider.setValue(int(100*value))
        self.settings.odom_res_slider.blockSignals(False)
        self.settings.odom_res_box.setValue(value)
        self.robot.odometer.res = value
        self.publish_sim_info_msg()

    # -----
    # LASER
    # -----
    def laser_combo_changed(self, value):
        if value == 0: # Custom
            self.toggle_enable_laser_settings(True)
        elif value == 1: # SICK LMS111
            self.laser_range_changed(d.SICK_111_RANGE)
            self.laser_min_bear_changed(d.SICK_111_MIN_ANGLE)
            self.laser_max_bear_changed(d.SICK_111_MAX_ANGLE)
            self.laser_res_changed(d.SICK_111_RES)
            self.laser_freq_changed(d.SICK_111_FREQ)
            self.laser_noise_changed(d.SICK_111_NOISE)
            self.toggle_enable_laser_settings(False)
        elif value == 2: # Hokuyo URG-04LX
            self.laser_range_changed(d.HOK_04_RANGE)
            self.laser_min_bear_changed(d.HOK_04_MIN_ANGLE)
            self.laser_max_bear_changed(d.HOK_04_MAX_ANGLE)
            self.laser_res_changed(d.HOK_04_RES)
            self.laser_freq_changed(d.HOK_04_FREQ)
            self.laser_noise_changed(d.HOK_04_NOISE)
            self.toggle_enable_laser_settings(False)

    def laser_freq_changed(self, value):
        self.settings.laser_freq_slider.setValue(value)
        self.settings.laser_freq_box.setValue(value)
        self.robot.laser.freq = value
        self.main.graphics_view.set_timer_frequencies()

    def laser_max_bear_changed(self, value):
        self.settings.laser_max_bear_box.setValue(value)
        self.settings.laser_max_bear_slider.setValue(value)
        min_value = self.settings.laser_min_bear_slider.value()
        if value > min_value:
            self.robot.laser.max_angle = value
        else:
            self.robot.laser.min_angle = value

    def laser_min_bear_changed(self, value):
        self.settings.laser_min_bear_box.setValue(value)
        self.settings.laser_min_bear_slider.setValue(value)
        max_value = self.settings.laser_max_bear_slider.value()
        if value < max_value:
            self.robot.laser.min_angle = value
        else:
            self.robot.laser.max_angle = value

    def laser_noise_changed(self, value):
        self.settings.laser_noise_slider.setValue(value)
        self.settings.laser_noise_box.setValue(value)
        self.robot.laser.noise = value/100.0 # convert to metres
        self.publish_sim_info_msg()

    def laser_range_changed(self, value):
        self.settings.laser_range_box.setValue(value)
        self.settings.laser_range_slider.setValue(value)
        self.robot.laser.range = value

    def laser_res_changed(self, value):
        self.settings.laser_res_slider.setValue(int(10*value))
        self.settings.laser_res_box.setValue(value)
        self.robot.laser.resolution = value

    # -------
    # MAPPING
    # -------
    def clear_map(self):
        self.main.graphics_view.delete_map()

    def load_map(self):
        self.settings.map_check.setCheckState(QtCore.Qt.Checked)
        self.main.graphics_view.draw_map_from_file('maps/random_map.txt')

    # -------
    # SETTINGS
    # -------
    def open_settings_dialog(self):
        self.dialog.exec_()

    # -------
    # ROS
    # -------
    def publish_sim_info_msg(self):
        msg = SimInfo()
        msg.robot_wheel_radius = self.robot.wheel_rad
        msg.robot_axle_track = self.robot.wheelbase
        msg.laser_noise = self.robot.laser.noise
        msg.encoder_resolution = self.robot.odometer.res
        msg.encoder_noise = self.robot.odometer.noise
        msg.gyro_noise = 0.0
        msg.compass_noise = 0.0
        msg.header.stamp = rospy.Time.now()
        self.sim_info_publisher.publish(msg)

    # -----
    # OTHER
    # -----

    def ang_vel_inc_changed(self, value):
        self.settings.ang_vel_inc_box.setValue(value)
        self.ang_vel_inc = value

    def keyPressEvent(self, event):
        """Adjusts the translational and angular velocites of the robot. Does
        not allow the velocities to go beyond the maximum and minimums. Sets
        the velocities to zero at zero-crossings (sign changes)."""
        vel_inc = self.vel_inc
        ang_vel_inc = self.ang_vel_inc * math.pi/180

        # W KEY PRESSED: INCREASE VELOCITY
        if event.key() == QtCore.Qt.Key_W:
            # Go to zero on sign changes
            if self.robot.vel < 0 and self.robot.vel + vel_inc > 0:
                self.robot.vel = 0
            # Change only if limit is not exceeded
            elif self.robot.vel < self.robot.max_vel:
                # Go to limit if increment would go beyond it
                if self.robot.vel + vel_inc > self.robot.max_vel:
                    self.robot.vel = self.robot.max_vel
                else:
                    self.robot.vel += vel_inc
                
        # S KEY PRESSED: DECREASE VELOCITY
        elif event.key() == QtCore.Qt.Key_S:
            # Go to zero on sign changes
            if self.robot.vel > 0 and self.robot.vel - vel_inc < 0:
                self.robot.vel = 0
            # Change only if limit is not exceeded
            elif self.robot.vel > -self.robot.max_vel:
                # Go to limit if increment would go beyond it
                if self.robot.vel - vel_inc < -self.robot.max_vel:
                    self.robot.vel = -self.robot.max_vel
                else:
                    self.robot.vel -= vel_inc

        # A KEY PRESSED: INCREASE ANGULAR VELOCITY
        elif event.key() == QtCore.Qt.Key_A:
            # Go to zero on sign changes
            if self.robot.ang_vel < 0 and self.robot.ang_vel + ang_vel_inc > 0:
                self.robot.ang_vel = 0
            # Change only if limit is not exceeded
            elif self.robot.ang_vel < self.robot.max_ang_vel:
                # Go to limit if increment would go beyond it
                if (self.robot.ang_vel + ang_vel_inc > self.robot.max_ang_vel):
                    self.robot.ang_vel = self.robot.max_ang_vel
                else:
                    self.robot.ang_vel += ang_vel_inc

        # D KEY PRESSED: DECREASE ANGULAR VELOCITY
        elif event.key() == QtCore.Qt.Key_D:
            # Go to zero on sign changes
            if self.robot.ang_vel > 0 and self.robot.ang_vel - ang_vel_inc < 0:
                self.robot.ang_vel = 0
            # Change only if limit is not exceeded
            elif self.robot.ang_vel > -self.robot.max_ang_vel:
                # Go to limit if increment would go beyond it
                if (self.robot.ang_vel - ang_vel_inc < -self.robot.max_ang_vel):
                    self.robot.ang_vel = -self.robot.max_ang_vel
                else:
                    self.robot.ang_vel -= ang_vel_inc

        # SPACE BAR PRESSED: STOP
        elif event.key() == QtCore.Qt.Key_Space:
            self.robot.vel = 0
            self.robot.ang_vel = 0

    def laser_check_changed(self, value):
        self.main.graphics_view.show_beams = value

    def map_check_changed(self, value):
        self.main.graphics_view.toggle_map(value)

    def vel_inc_changed(self, value):
        self.settings.vel_inc_box.setValue(value)
        self.vel_inc = value

    # --------------------------------------------------------------------------
    # UPDATE/CHANGE GUI ELEMENTS
    # --------------------------------------------------------------------------

    def toggle_enable_laser_settings(self, b):
        self.settings.laser_range_slider.setEnabled(b)
        self.settings.laser_range_box.setEnabled(b)
        self.settings.laser_min_bear_slider.setEnabled(b)
        self.settings.laser_max_bear_slider.setEnabled(b)
        self.settings.laser_min_bear_box.setEnabled(b)
        self.settings.laser_max_bear_box.setEnabled(b)
        self.settings.laser_res_slider.setEnabled(b)
        self.settings.laser_res_box.setEnabled(b)
        self.settings.laser_freq_slider.setEnabled(b)
        self.settings.laser_freq_box.setEnabled(b)
        self.settings.laser_noise_slider.setEnabled(b)
        self.settings.laser_noise_box.setEnabled(b)

    def toggle_enable_robot_settings(self, b):
        self.settings.robot_length_slider.setEnabled(b)
        self.settings.robot_length_box.setEnabled(b)
        self.settings.robot_width_slider.setEnabled(b)
        self.settings.robot_width_box.setEnabled(b)
        self.settings.robot_wheel_rad_slider.setEnabled(b)
        self.settings.robot_wheel_rad_box.setEnabled(b)
        self.settings.robot_wheelbase_slider.setEnabled(b)
        self.settings.robot_wheelbase_box.setEnabled(b)
        self.settings.robot_vel_slider.setEnabled(b)
        self.settings.robot_vel_box.setEnabled(b)
        self.settings.robot_ang_vel_slider.setEnabled(b)
        self.settings.robot_ang_vel_box.setEnabled(b)

    def update_info_labels(self):
        self.main.pose_label.setText("%0.2f m, %0.2f m, %d deg" % (self.robot.x,
                self.robot.y, 180/math.pi*self.robot.heading))
        self.main.velocity_label.setText("%0.2f m/s" % self.robot.vel)
        self.main.ang_vel_label.setText("%d deg/s" % int(
            180/math.pi*self.robot.ang_vel))

    # --------------------------------------------------------------------------
    # UTILITY METHODS
    # --------------------------------------------------------------------------

    def settings_to_default(self):
        # -----
        # ROBOT
        # -----
        self.robot_length_changed(d.ROBOT_LENGTH)
        self.robot_width_changed(d.ROBOT_WIDTH)
        self.robot_wheel_rad_changed(d.ROBOT_WHEEL_RAD)
        self.robot_wheelbase_changed(d.ROBOT_WHEELBASE)
        self.robot_vel_changed(d.ROBOT_MAX_VEL)
        self.robot_ang_vel_changed(d.ROBOT_MAX_ANG_VEL)

        # --------
        # ODOMETER
        # --------
        self.odom_res_changed(d.ODOM_RES)
        self.odom_freq_changed(d.ODOM_FREQ)
        self.odom_noise_changed(d.ODOM_NOISE)

        # -----
        # LASER
        # -----
        self.laser_range_changed(d.LASER_RANGE)
        self.laser_min_bear_changed(d.LASER_MIN_ANGLE)
        self.laser_max_bear_changed(d.LASER_MAX_ANGLE)
        self.laser_res_changed(d.LASER_RES)
        self.laser_freq_changed(d.LASER_FREQ)
        self.laser_noise_changed(d.LASER_NOISE)

        # -----
        # OTHER
        # -----
        self.vel_inc_changed(d.VELOCITY_INCREMENT)
        self.ang_vel_inc_changed(d.ANG_VELOCITY_INCREMENT)

    def zoom_in(self):
        self.main.graphics_view.set_scale(1.1)

    def zoom_out(self):
        self.main.graphics_view.set_scale(0.9)


class PlotGraphicsView(QtGui.QGraphicsView):
    def __init__(self, parent):
        super(PlotGraphicsView, self).__init__(parent)
        self.parent = parent
        self.plot_freq = d.PLOT_FREQ
        self.line_item = None # current line user is drawing
        self.zoom_scale = 20
        self.scale(self.zoom_scale, -self.zoom_scale)
        self.set_colours()
        self.g_scene = QtGui.QGraphicsScene(self)
        self.setScene(self.g_scene)
        self.draw_scale()
        self.poly_item = None
        # Containers
        self.line_map = [] # line properties 
        self.line_item_map = [] # line graphic items
        self.obstacle_items = [] # obstacle polygon items
        # Flags
        self.draw_mode = 'freehand' # freehand, line, poly
        self.drawing_line = False # user is currently drawing a line
        self.freehand = False
        self.show_beams = True # laser beams (not just hits) are shown
        # Timers
        self.plot_timer = QtCore.QTimer()
        self.odom_timer = QtCore.QTimer()
        self.laser_timer = QtCore.QTimer()
        # ROS
        rospy.init_node('msl_sim')
        self.compass_publisher = rospy.Publisher('/msl_sim/compass', Compass, queue_size=10)
        self.encoders_publisher = rospy.Publisher('/msl_sim/encoders', Encoders, queue_size=10)
        self.gyro = rospy.Publisher('/msl_sim/gyro', Gyro, queue_size=10)
        self.laser_publisher = rospy.Publisher('/msl_sim/scan', LaserScan, queue_size=10)

    # --------------------------------------------------------------------------
    # SETUP METHODS
    # --------------------------------------------------------------------------
    def draw_scale(self):
        scale_line_horiz = QtGui.QGraphicsLineItem(-26, 0, 26, 0)
        scale_line_horiz.setPen(self.scale_pen)
        self.scene().addItem(scale_line_horiz)
        scale_line_vert = QtGui.QGraphicsLineItem(0, -26, 0, 26)
        scale_line_vert.setPen(self.scale_pen)
        self.scene().addItem(scale_line_vert)
        font = QtGui.QFont('Monospace', pointSize=12)
        for i in range(-26, 27, 2):
            if i == 0:
                continue
            # Horizontal scale
            h_tick = QtGui.QGraphicsLineItem(i, -0.3, i, 0.3)
            h_tick.setPen(self.scale_pen)
            h_text = self.scene().addText('%d' % i, font=font)
            if i > 0:
                h_text.setPos(i + h_text.textWidth()/2.0 + 0.15, -0.25)
            else:
                h_text.setPos(i + h_text.textWidth()/2.0, -0.25)
            h_text.scale(1.0/self.zoom_scale, -1.0/self.zoom_scale)
            h_text.setDefaultTextColor(QtGui.QColor(210, 210, 210))
            self.scene().addItem(h_tick)
            # Vertical scale
            v_tick = QtGui.QGraphicsLineItem(-0.3, i, 0.3, i)
            v_tick.setPen(self.scale_pen)
            v_text = self.scene().addText('%d' % i, font=font)
            v_text.setPos(0.3, i + 0.55)
            v_text.scale(1.0/self.zoom_scale, -1.0/self.zoom_scale)
            v_text.setDefaultTextColor(QtGui.QColor(210, 210, 210))
            self.scene().addItem(v_tick)

    def initialiseRobot(self):
        """Draws the robot in the scene."""
        self.rect = self.scene().addRect(self.robot.x - self.robot.length/2.0,
                                         self.robot.y - self.robot.width/2.0,
                                         self.robot.length, self.robot.width,
                                         self.robot_pen)
        self.rect.setTransformOriginPoint(self.robot.x,
                                          self.robot.y)
        self.rect.setZValue(15)
        self.rect.setBrush(QtGui.QColor(135,236,250))
        self.previous_pose = self.robot.pose

    def set_colours(self):
        """Creates QPen instances for all the objects that will be plotted."""
        self.laser_pen = QtGui.QPen()
        self.robot_pen = QtGui.QPen()
        self.scale_pen = QtGui.QPen()
        dark_red = QtGui.QColor(204, 0, 0)
        blue = QtGui.QColor(0, 0, 255)
        gray = QtGui.QColor(210, 210, 210)
        self.laser_pen.setColor(dark_red)
        self.robot_pen.setColor(blue)
        self.scale_pen.setColor(gray)

    # --------------------------------------------------------------------------
    # TIMER METHODS
    # --------------------------------------------------------------------------
    def laser_update(self):
        ranges = self.robot.scan_laser(self.line_map)
        self.publish_laser_msg(ranges)
        self.latest_laser_scan = ranges

    def move_zoomed_view(self):
        # Adjust the window of the zoomed in view
        self.zoom.setSceneRect(self.robot.x - self.robot.length/2.0, 
                self.robot.y - self.robot.width/2.0, 
                self.robot.length, self.robot.width)
        heading_change = 180/math.pi * (self.previous_pose[2] - self.robot.heading)
        self.zoom.rotate(heading_change)
        self.previous_pose = self.robot.pose

    def odometry_update(self):
        odom = self.robot.update_pose()

    def plot_update(self):
        """Updates the plot. This method is called automatically by the
        plot_timer."""
        # Laser beams
        if self.robot.scanned:
            self.draw_laser_beams(self.latest_laser_scan)
            # Reset flag
            self.robot.scanned = False
        # Robot
        if self.robot.changed:
            self.rect.setRect(self.robot.x - self.robot.length/2.0,
                    self.robot.y - self.robot.width/2.0,
                    self.robot.length, self.robot.width)
            self.rect.setTransformOriginPoint(self.robot.x,
                                              self.robot.y)
            self.rect.setRotation(180/math.pi * self.robot.heading)
            # Camera pose for zoomed in view
            self.move_zoomed_view()
            # Reset flag
            self.robot.changed = False

    def set_timer_frequencies(self):
        self.plot_timer.setInterval(1000.0/self.plot_freq)
        self.odom_timer.setInterval(1000.0/self.robot.odometer.freq)
        self.laser_timer.setInterval(1000.0/self.robot.laser.freq)

    def start_timers(self):
        """Starts separate timers to update the plot, odometry, and range data
        from the laser."""
        self.set_timer_frequencies()
        self.plot_timer.timeout.connect(self.plot_update)
        self.odom_timer.timeout.connect(self.odometry_update)
        self.laser_timer.timeout.connect(self.laser_update)
        self.plot_timer.start()
        self.odom_timer.start()
        self.laser_timer.start()

    # --------------------------------------------------------------------------
    # DRAWING METHODS
    # --------------------------------------------------------------------------
    def delete_map(self):
        for item in self.line_item_map:
            self.scene().removeItem(item)
        for item in self.obstacle_items:
            self.scene().removeItem(item)
        self.line_item_map = []
        self.obstacle_items = []
        self.line_map = []

    def draw_laser_beams(self, ranges):
        """Deletes the previously drawn laser polygon and plots the latest laser
        measurement."""
        # Delete the laser polygon from the scene
        if self.poly_item:
            self.scene().removeItem(self.poly_item)
            self.poly_item = None
        if not self.show_beams:
            return
        x, y, theta = self.robot.pose
        laser_range = self.robot.laser.range
        laser_min_angle = self.robot.laser.min_angle
        laser_res = self.robot.laser.resolution
        polygon = QtGui.QPolygonF()
        polygon.append(QtCore.QPointF(self.robot.x, self.robot.y))
        for (i, r) in enumerate(ranges):
            if r == 0:
                r = laser_range
            # Get the coordinates of the laser detected point
            x_2 = x + math.pi/180 + r*math.cos(
                    theta + math.pi/180*(laser_min_angle + laser_res*i))
            y_2 = y + math.pi/180 + r*math.sin(
                    theta + math.pi/180*(laser_min_angle + laser_res*i))
            polygon.append(QtCore.QPointF(x_2, y_2))
        self.poly_item = QtGui.QGraphicsPolygonItem(polygon, scene=self.scene())
        self.poly_item.setZValue(5)
        self.poly_item.setPen(self.laser_pen)
        beam_color = QtGui.QColor(255, 0, 0)
        beam_color.setAlpha(40)
        self.poly_item.setBrush(beam_color)

    def draw_map_from_file(self, filename):
        with open(filename, 'r') as f:
            for line in f:
                if line[0] != '#':
                    line.strip()
                    coord_list = line.split(' ')
                    x_1, y_1, x_2, y_2 = [float(num) for num in coord_list]
                    line_item = QtGui.QGraphicsLineItem(x_1, y_1, x_2, y_2)
                    line_item.setZValue(10)
                    self.scene().addItem(line_item)
                    self.line_item_map.append(line_item)
                    line = mod.get_line_dict(x_1, y_1, x_2, y_2)
                    self.line_map.append(line)
    
    def draw_polygon(self, x, y, num_edges, diameter, angle):
        poly = QtGui.QPolygonF()
        vertices = []
        for i in range(num_edges):
            theta = i * 2*math.pi/num_edges
            x_v = x + diameter/2.0 * math.cos(theta)
            y_v = y + diameter/2.0 * math.sin(theta)
            poly.append(QtCore.QPointF(x_v, y_v))
            vertices.append((x_v, y_v))
        obstacle = QtGui.QGraphicsPolygonItem(poly, scene=self.scene())
        obstacle.setZValue(10)
        obs_color = QtGui.QColor(220, 220, 220)
        obstacle.setBrush(obs_color)
        self.obstacle_items.append(obstacle)
        # Add lines making up polygon to line map
        for ind, vert in enumerate(vertices):
            x_1, y_1 = vert
            x_2, y_2 = vertices[ind-1]
            line = mod.get_line_dict(x_1, y_1, x_2, y_2)
            self.line_map.append(line)

    def toggle_map(self, value):
        for item in self.line_item_map:
            item.setVisible(value)
        for item in self.obstacle_items:
            item.setVisible(value)
    # --------------------------------------------------------------------------
    # ROS METHODS
    # --------------------------------------------------------------------------
    def publish_laser_msg(self, ranges):
        msg = LaserScan()
        msg.angle_min = self.robot.laser.min_angle
        msg.angle_max = self.robot.laser.max_angle
        msg.angle_increment = self.robot.laser.resolution
        msg.range_min = 0.0
        msg.range_max = self.robot.laser.range
        msg.ranges = ranges
        msg.header.stamp = rospy.Time.now()
        self.laser_publisher.publish(msg)

    # --------------------------------------------------------------------------
    # UTILITY METHODS
    # --------------------------------------------------------------------------
    def set_scale(self, value):
        self.scale(value, value)

class PlotGraphicsViewZoom(QtGui.QGraphicsView):
    def __init__(self, parent):
        super(PlotGraphicsViewZoom, self).__init__(parent)
        self.parent = parent
        self.scale(40, -40)

    def initializeCamera(self, robot):
        self.setSceneRect(robot.x - robot.length/2.0, robot.y - robot.width/2.0,
                robot.length, robot.width)
        self.rotate(90)
