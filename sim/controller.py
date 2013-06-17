# Python imports
import math
import time
import random

# PySide imports
from PySide import QtGui, QtCore

# MSL Sim imports
import sim.model as mod
import sim.defaults as d


class MainWindow(QtGui.QMainWindow):
    """The main window. This window displays all the widgets."""
    def __init__(self):
        super(MainWindow, self).__init__()
        self.robot = mod.Robot()
        self.loadGUI()
        # Place and scale the logo
        pixmap = QtGui.QPixmap("msl_logo.png")
        self.ui.logo_label.setPixmap(pixmap)
        # Give the zoomed in plotting area the same scene as the zoomed out one
        self.ui.graphics_view_zoom.setScene(self.ui.graphics_view.scene())
        # Initialize the camera in the zoomed in plotting window
        self.ui.graphics_view_zoom.initializeCamera(self.robot)
        # Give the zoomed-out plotting area a copy of the zoomed-in plotting
        # area so it can change it based on its timers
        self.ui.graphics_view.zoom = self.ui.graphics_view_zoom
        # Give the plotting area the same robot as the rest of the GUI and start
        # updating it via timers
        self.ui.graphics_view.robot = self.robot
        self.ui.graphics_view.initialiseRobot()
        self.settings_to_default()
        # Start a timer that updates the labels
        self.label_timer = QtCore.QTimer()
        self.label_timer.timeout.connect(self.update_info_labels)
        self.label_timer.start()
        # Establish a timer for recordings
        self.record_timer = QtCore.QTimer()
        self.record_timer.setInterval(100)
        self.record_timer.timeout.connect(self.update_record_timer)
        # Start timers that update the plot and the model
        self.ui.graphics_view.start_timers()

    # --------------------------------------------------------------------------
    # SETUP METHODS
    # --------------------------------------------------------------------------
    def connect_signals_to_slots(self):
        # -----
        # ROBOT
        # -----
        self.ui.robot_ang_vel_box.valueChanged.connect(
                self.robot_ang_vel_changed)
        self.ui.robot_ang_vel_slider.valueChanged.connect(
                self.robot_ang_vel_changed)
        self.ui.robot_combo.currentIndexChanged.connect(
                self.robot_combo_changed)
        self.ui.robot_length_box.valueChanged.connect(
                self.robot_length_changed)
        self.ui.robot_length_slider.valueChanged.connect(
                lambda: self.robot_length_changed(
                    self.ui.robot_length_slider.value()/10.0))
        self.ui.robot_wheel_rad_box.valueChanged.connect(
                self.robot_wheel_rad_changed)
        self.ui.robot_wheel_rad_slider.valueChanged.connect(
                lambda: self.robot_wheel_rad_changed(
                    self.ui.robot_wheel_rad_slider.value()/100.0))
        self.ui.robot_wheelbase_box.valueChanged.connect(
                self.robot_wheelbase_changed)
        self.ui.robot_wheelbase_slider.valueChanged.connect(
                lambda: self.robot_wheelbase_changed(
                    self.ui.robot_wheelbase_slider.value()/10.0))
        self.ui.robot_width_box.valueChanged.connect(
                self.robot_width_changed)
        self.ui.robot_width_slider.valueChanged.connect(
                lambda: self.robot_width_changed(
                    self.ui.robot_width_slider.value()/10.0))
        self.ui.robot_vel_box.valueChanged.connect(self.robot_vel_changed)
        self.ui.robot_vel_slider.valueChanged.connect(
                lambda: self.robot_vel_changed(
                    self.ui.robot_vel_slider.value()/10.0))

        # --------
        # ODOMETER
        # --------
        self.ui.odom_freq_box.valueChanged.connect(self.odom_freq_changed)
        self.ui.odom_freq_slider.valueChanged.connect(self.odom_freq_changed)
        self.ui.odom_noise_box.valueChanged.connect(self.odom_noise_changed)
        self.ui.odom_noise_slider.valueChanged.connect(
                lambda: self.odom_noise_changed(
                    self.ui.odom_noise_slider.value()/10.0))
        self.ui.odom_res_box.valueChanged.connect(self.odom_res_changed)
        self.ui.odom_res_slider.valueChanged.connect(
                lambda: self.odom_res_changed(
                    self.ui.odom_res_slider.value()/100.0))

        # -----
        # LASER
        # -----
        self.ui.laser_combo.currentIndexChanged.connect(self.laser_combo_changed)
        self.ui.laser_freq_box.valueChanged.connect(
                self.laser_freq_changed)
        self.ui.laser_freq_slider.valueChanged.connect(
                self.laser_freq_changed)
        self.ui.laser_max_bear_box.valueChanged.connect(
                self.laser_max_bear_changed)
        self.ui.laser_max_bear_slider.valueChanged.connect(
                self.laser_max_bear_changed)
        self.ui.laser_min_bear_box.valueChanged.connect(
                self.laser_min_bear_changed)
        self.ui.laser_min_bear_slider.valueChanged.connect(
                self.laser_min_bear_changed)
        self.ui.laser_noise_box.valueChanged.connect(
                self.laser_noise_changed)
        self.ui.laser_noise_slider.valueChanged.connect(
                self.laser_noise_changed)
        self.ui.laser_range_box.valueChanged.connect(
                self.laser_range_changed)
        self.ui.laser_range_slider.valueChanged.connect(
                self.laser_range_changed)
        self.ui.laser_res_box.valueChanged.connect(
                self.laser_res_changed)
        self.ui.laser_res_slider.valueChanged.connect(
                lambda: self.laser_res_changed(
                    self.ui.laser_res_slider.value()/10.0))

        # -----
        # MAPPING
        # -----
        self.ui.clear_map_button.clicked.connect(self.clear_map)
        self.ui.draw_polygon_button.clicked.connect(self.draw_polygon_mode)
        self.ui.generate_map_button.clicked.connect(self.generate_map)
        self.ui.load_map_button.clicked.connect(self.load_map)
        self.ui.mean_diameter_box.valueChanged.connect(
                self.mean_diameter_changed)
        self.ui.mean_diameter_slider.valueChanged.connect(
                lambda: self.mean_diameter_changed(
                    self.ui.mean_diameter_slider.value()/100.0))
        self.ui.std_dev_diameter_box.valueChanged.connect(
                self.std_dev_diameter_changed)
        self.ui.std_dev_diameter_slider.valueChanged.connect(
                lambda: self.std_dev_diameter_changed(
                    self.ui.std_dev_diameter_slider.value()/100.0))
        self.ui.num_obstacles_slider.valueChanged.connect(
                self.num_obstacles_changed)
        self.ui.num_obstacles_box.valueChanged.connect(
                self.num_obstacles_changed)
        self.ui.polygon_edges_combo.currentIndexChanged.connect(
                self.polygon_edges_combo_changed)
        self.ui.polygon_size_box.valueChanged.connect(self.polygon_size_changed)
        self.ui.polygon_size_slider.valueChanged.connect(
                lambda: self.polygon_size_changed(
                    self.ui.polygon_size_slider.value()/100.0))

        # -----
        # OTHER
        # -----
        self.ui.ang_vel_inc_box.valueChanged.connect(self.ang_vel_inc_changed)
        self.ui.laser_check.stateChanged.connect(self.laser_check_changed)
        self.ui.map_check.stateChanged.connect(self.map_check_changed)
        self.ui.toggle_record_button.clicked.connect(self.toggle_recording)
        self.ui.vel_inc_box.valueChanged.connect(self.vel_inc_changed)
        self.ui.zoom_in_button.clicked.connect(self.zoom_in)
        self.ui.zoom_out_button.clicked.connect(self.zoom_out)


    def loadGUI(self):
        """Load the GUI from the .py file that was generated by the .ui file
        using the pyside-uic tool."""
        # import generated .py file here to prevent circular reference
        from sim.view import Ui_main_window
        self.ui = Ui_main_window()
        self.ui.setupUi(self)
        self.connect_signals_to_slots()
        # Add items to pull down menus
        self.ui.laser_combo.addItems(["Custom", "SICK LMS111",
            "Hokuyo URG-04LX"])
        self.ui.robot_combo.addItems(["Custom", "Clearpath Husky A200",
            "MobileRobots P3AT"])

    # --------------------------------------------------------------------------
    # SLOTS
    # --------------------------------------------------------------------------

    # -----
    # ROBOT
    # -----
    def robot_ang_vel_changed(self, value):
        self.ui.robot_ang_vel_slider.setValue(value)
        self.ui.robot_ang_vel_box.setValue(value)
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
        self.ui.robot_length_slider.setValue(int(10*value))
        self.ui.robot_length_box.setValue(value)
        self.robot.set_length(value)

    def robot_wheel_rad_changed(self, value):
        self.ui.robot_wheel_rad_slider.setValue(int(100*value))
        self.ui.robot_wheel_rad_box.setValue(value)
        self.robot.wheel_rad = value

    def robot_wheelbase_changed(self, value):
        self.ui.robot_wheelbase_slider.setValue(int(10*value))
        self.ui.robot_wheelbase_box.setValue(value)
        self.robot.wheelbase = value

    def robot_width_changed(self, value):
        self.ui.robot_width_slider.setValue(int(10*value))
        self.ui.robot_width_box.setValue(value)
        self.robot.set_width(value)

    def robot_vel_changed(self, value):
        self.ui.robot_vel_slider.setValue(int(10*value))
        self.ui.robot_vel_box.setValue(value)
        self.robot.max_vel = value

    # --------
    # ODOMETER
    # --------
    def odom_freq_changed(self, value):
        self.ui.odom_freq_slider.setValue(value)
        self.ui.odom_freq_box.setValue(value)
        self.robot.odometer.freq = value

    def odom_noise_changed(self, value):
        self.ui.odom_noise_slider.setValue(int(10*value))
        self.ui.odom_noise_box.setValue(value)
        self.robot.odometer.noise = value

    def odom_res_changed(self, value):
        self.ui.odom_res_slider.blockSignals(True)
        self.ui.odom_res_slider.setValue(int(100*value))
        self.ui.odom_res_slider.blockSignals(False)
        self.ui.odom_res_box.setValue(value)
        self.robot.odometer.res = value

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
        self.ui.laser_freq_slider.setValue(value)
        self.ui.laser_freq_box.setValue(value)
        self.robot.laser.freq = value
        self.ui.graphics_view.set_timer_frequencies()

    def laser_max_bear_changed(self, value):
        self.ui.laser_max_bear_box.setValue(value)
        self.ui.laser_max_bear_slider.setValue(value)
        min_value = self.ui.laser_min_bear_slider.value()
        if value > min_value:
            self.robot.laser.max_angle = value
        else:
            self.robot.laser.min_angle = value

    def laser_min_bear_changed(self, value):
        self.ui.laser_min_bear_box.setValue(value)
        self.ui.laser_min_bear_slider.setValue(value)
        max_value = self.ui.laser_max_bear_slider.value()
        if value < max_value:
            self.robot.laser.min_angle = value
        else:
            self.robot.laser.max_angle = value

    def laser_noise_changed(self, value):
        self.ui.laser_noise_slider.setValue(value)
        self.ui.laser_noise_box.setValue(value)
        self.robot.laser.noise = value/100.0 # convert to metres

    def laser_range_changed(self, value):
        self.ui.laser_range_box.setValue(value)
        self.ui.laser_range_slider.setValue(value)
        self.robot.laser.range = value

    def laser_res_changed(self, value):
        self.ui.laser_res_slider.setValue(int(10*value))
        self.ui.laser_res_box.setValue(value)
        self.robot.laser.resolution = value

    # -------
    # MAPPING
    # -------
    def clear_map(self):
        self.ui.graphics_view.delete_map()

    def create_polygon(self, x, y, edges, diameter, angle):
        self.ui.graphics_view.draw_polygon(x, y, edges, diameter, angle)

    def draw_polygon_mode(self):
        self.ui.graphics_view.poly_edges = int(self.ui.polygon_edges_combo.currentText())
        self.ui.graphics_view.poly_diameter = self.ui.polygon_size_box.value()
        self.ui.graphics_view.poly_angle = self.ui.polygon_angle_box.value()
        self.ui.graphics_view.draw_mode = 'poly'

    def generate_map(self):
        self.clear_map()
        option = self.ui.polygons_combo.currentText()
        edges = int(self.ui.polygon_edges_combo.currentText())
        diameter = self.ui.polygon_size_box.value()
        angle = self.ui.polygon_angle_box.value()
        self.ui.map_check.setCheckState(QtCore.Qt.Checked)
        for i in range(self.ui.num_obstacles_box.value()):
            if option == 'Random':
                edges = random.choice([3, 4, 5, 6])
                mean_diameter = self.ui.mean_diameter_box.value()
                std_diameter = self.ui.std_dev_diameter_box.value()
                diameter = random.gauss(mean_diameter, std_diameter)
                angle = random.randint(0, 180)
            x = random.uniform(-d.MAP_WIDTH/2.0, d.MAP_WIDTH/2.0)
            y = random.uniform(-d.MAP_HEIGHT/2.0, d.MAP_HEIGHT/2.0)
            self.create_polygon(x, y, edges, diameter, angle)

    def load_map(self):
        self.ui.map_check.setCheckState(QtCore.Qt.Checked)
        self.ui.graphics_view.draw_map_from_file('maps/random_map.txt')

    def mean_diameter_changed(self, value):
        self.ui.mean_diameter_slider.blockSignals(True)
        self.ui.mean_diameter_slider.setValue(int(100*value))
        self.ui.mean_diameter_slider.blockSignals(False)
        self.ui.mean_diameter_box.setValue(value)

    def num_obstacles_changed(self, value):
        self.ui.num_obstacles_box.setValue(value)
        self.ui.num_obstacles_slider.setValue(value)

    def polygon_edges_combo_changed(self, value):
        self.ui.graphics_view.poly_edges = int(self.ui.polygon_edges_combo.currentText())

    def polygon_size_changed(self, value):
        self.ui.polygon_size_slider.blockSignals(True)
        self.ui.polygon_size_slider.setValue(int(value * 100))
        self.ui.polygon_size_slider.blockSignals(False)
        self.ui.polygon_size_box.setValue(value)
        self.ui.graphics_view.poly_diameter = value

    def std_dev_diameter_changed(self, value):
        self.ui.std_dev_diameter_slider.blockSignals(True)
        self.ui.std_dev_diameter_slider.setValue(int(100*value))
        self.ui.std_dev_diameter_slider.blockSignals(False)
        self.ui.std_dev_diameter_box.setValue(value)

    # -----
    # OTHER
    # -----

    def ang_vel_inc_changed(self, value):
        self.ui.ang_vel_inc_box.setValue(value)
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
        self.ui.graphics_view.show_beams = value

    def map_check_changed(self, value):
        self.ui.graphics_view.toggle_map(value)

    def vel_inc_changed(self, value):
        self.ui.vel_inc_box.setValue(value)
        self.vel_inc = value

    # --------------------------------------------------------------------------
    # UPDATE/CHANGE GUI ELEMENTS
    # --------------------------------------------------------------------------

    def toggle_enable_laser_settings(self, b):
        self.ui.laser_range_slider.setEnabled(b)
        self.ui.laser_range_box.setEnabled(b)
        self.ui.laser_min_bear_slider.setEnabled(b)
        self.ui.laser_max_bear_slider.setEnabled(b)
        self.ui.laser_min_bear_box.setEnabled(b)
        self.ui.laser_max_bear_box.setEnabled(b)
        self.ui.laser_res_slider.setEnabled(b)
        self.ui.laser_res_box.setEnabled(b)
        self.ui.laser_freq_slider.setEnabled(b)
        self.ui.laser_freq_box.setEnabled(b)
        self.ui.laser_noise_slider.setEnabled(b)
        self.ui.laser_noise_box.setEnabled(b)

    def toggle_enable_robot_settings(self, b):
        self.ui.robot_length_slider.setEnabled(b)
        self.ui.robot_length_box.setEnabled(b)
        self.ui.robot_width_slider.setEnabled(b)
        self.ui.robot_width_box.setEnabled(b)
        self.ui.robot_wheel_rad_slider.setEnabled(b)
        self.ui.robot_wheel_rad_box.setEnabled(b)
        self.ui.robot_wheelbase_slider.setEnabled(b)
        self.ui.robot_wheelbase_box.setEnabled(b)
        self.ui.robot_vel_slider.setEnabled(b)
        self.ui.robot_vel_box.setEnabled(b)
        self.ui.robot_ang_vel_slider.setEnabled(b)
        self.ui.robot_ang_vel_box.setEnabled(b)

    def update_info_labels(self):
        self.ui.pose_label.setText("%0.2f m, %0.2f m, %d deg" % (self.robot.x,
                self.robot.y, 180/math.pi*self.robot.heading))
        self.ui.velocity_label.setText("%0.2f m/s" % self.robot.vel)
        self.ui.ang_vel_label.setText("%d deg/s" % int(
            180/math.pi*self.robot.ang_vel))

    def update_record_timer(self):
        elapsed_time = time.time() - self.ui.graphics_view.record_start_time
        self.ui.record_timer_display.setText(
                '%0.1f s' % elapsed_time)
        self.ui.record_laser_meas_label.setText(
                '%d' % len(self.ui.graphics_view.laser_history))
        self.ui.record_odom_meas_label.setText(
                '%d' % len(self.ui.graphics_view.odom_history))

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

    def toggle_recording(self):
        self.ui.graphics_view.toggle_recording()
        if self.ui.graphics_view.recording:
            self.record_timer.start()
            self.ui.toggle_record_button.setText('Stop Recording')
        else:
            self.record_timer.stop()
            self.ui.toggle_record_button.setText('Start Recording')

    def zoom_in(self):
        self.ui.graphics_view.set_scale(1.1)

    def zoom_out(self):
        self.ui.graphics_view.set_scale(0.9)


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
        self.recording = False # data is being recorded
        self.show_beams = True # laser beams (not just hits) are shown
        # Timers
        self.plot_timer = QtCore.QTimer()
        self.odom_timer = QtCore.QTimer()
        self.laser_timer = QtCore.QTimer()

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
    # EVENTS
    # --------------------------------------------------------------------------
    def mousePressEvent(self, event):
        """Records the coordinates of the point where the mouse was clicked on
        the scene."""
        if event.button() == QtCore.Qt.LeftButton:
            click_pos = QtCore.QPointF(self.mapToScene(event.pos()))
            if self.draw_mode == 'poly':
                x = click_pos.x()
                y = click_pos.y()
                self.draw_polygon(x, y, self.poly_edges, self.poly_diameter,
                        self.poly_angle)
            elif self.draw_mode == 'freehand':
                self.start = click_pos
            elif self.draw_mode == 'line':
                pass

    def mouseMoveEvent(self, event):
        """Draws a line between the start and the current position of the 
        cursor."""
        if event.buttons() == QtCore.Qt.LeftButton:
            if self.draw_mode == 'freehand':
                if self.drawing_line:
                    end = QtCore.QPointF(self.mapToScene(event.pos()))
                    self.line_item.setLine(QtCore.QLineF(self.start, end))
                else:
                    end = QtCore.QPointF(self.mapToScene(event.pos()))
                    self.line_item = QtGui.QGraphicsLineItem(
                            QtCore.QLineF(self.start, end))
                    self.line_item.setZValue(10)
                    self.scene().addItem(self.line_item)
                    self.drawing_line = True

    def mouseReleaseEvent(self, event):
        """Draws a line from the recorded click coordinates and the position of 
        the cursor when the left mouse button is released. Adds this line to
        the line map."""
        if event.button() == QtCore.Qt.LeftButton:
            if self.draw_mode == 'freehand':
                end = QtCore.QPointF(self.mapToScene(event.pos()))
                # Only draw a line if the start and end coordinates are different
                if self.start.x() != end.x() or self.start.y() != end.y():
                    line = mod.get_line_dict(self.start.x(), self.start.y(),
                            end.x(), end.y())
                    self.line_map.append(line)
                    self.line_item_map.append(self.line_item)
                self.drawing_line = False
                self.line_item = None

    # --------------------------------------------------------------------------
    # TIMER METHODS
    # --------------------------------------------------------------------------
    def laser_update(self):
        ranges = self.robot.scan_laser(self.line_map)
        self.latest_laser_scan = ranges
        if self.recording:
            elapsed_time = time.time() - self.record_start_time
            self.laser_history.append(('ranges', elapsed_time, ranges))

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
        if self.recording:
            elapsed_time = time.time() - self.record_start_time
            self.odom_history.append(('odom', elapsed_time, odom))

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
        self.scale(0.9, 0.9)
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
    # FILE METHODS
    # --------------------------------------------------------------------------
    def write_file_header(self, f):
        f.write('### SETTINGS ###\n\n')
        f.write('# ROBOT #\n')
        f.write('robot_width %0.3f\n' % self.robot.width)
        f.write('robot_length %0.3f\n' % self.robot.length)
        f.write('robot_wheelbase %0.3f\n' % self.robot.wheelbase)
        f.write('robot_wheel_rad %0.3f\n' % self.robot.wheel_rad)
        f.write('robot_max_vel %0.3f\n' % self.robot.max_vel)
        f.write('robot_max_ang_vel %0.3f\n' % self.robot.max_ang_vel)
        f.write('\n')
        f.write('# ODOMETERS #\n')
        f.write('odom_res %0.3f\n' % self.robot.odometer.res)
        f.write('odom_freq %d\n' % self.robot.odometer.freq)
        f.write('odom_noise %0.3f\n' % self.robot.odometer.noise)
        f.write('\n')
        f.write('# LASER #\n')
        f.write('laser_min_angle %d\n' % self.robot.laser.min_angle)
        f.write('laser_max_angle %d\n' % self.robot.laser.max_angle)
        f.write('laser_res %0.3f\n' % self.robot.laser.resolution)
        f.write('laser_range %0.3f\n' % self.robot.laser.range)
        f.write('laser_noise %0.3f\n' % self.robot.laser.noise)
        f.write('laser_freq %d\n' % self.robot.laser.freq)
        f.write('\n')

    def write_record_file(self):
        history = self.odom_history + self.laser_history
        chrono_hist = sorted(history, key = lambda history: history[1])
        filename = time.strftime("sim-%Y%m%d-%H%M%S") + '.txt'
        with open('data/' + filename, 'w+') as f:
            self.write_file_header(f)
            f.write('### DATA ###\n\n')
            for kind, t, data in chrono_hist:
                if kind == 'odom':
                    f.write('%s %0.4f %0.5f %0.5f\n' % (kind, t, data[0],
                        data[1]))
                elif kind == 'ranges':
                    f.write('%s %0.4f ' % (kind, t))
                    for r in data:
                        if r == 0:
                            f.write('%d ' % r)
                        else:
                            f.write('%0.3f ' % r)
                    f.write('\n')

    # --------------------------------------------------------------------------
    # UTILITY METHODS
    # --------------------------------------------------------------------------
    def set_scale(self, value):
        self.scale(value, value)

    def toggle_recording(self):
        if not self.recording:
            self.laser_history = []
            self.odom_history = []
            self.record_start_time = time.time()
            self.recording = True
        else:
            self.recording = False
            self.write_record_file()


class PlotGraphicsViewZoom(QtGui.QGraphicsView):
    def __init__(self, parent):
        super(PlotGraphicsViewZoom, self).__init__(parent)
        self.parent = parent
        self.scale(40, -40)

    def initializeCamera(self, robot):
        self.setSceneRect(robot.x - robot.length/2.0, robot.y - robot.width/2.0,
                robot.length, robot.width)
        self.rotate(90)
