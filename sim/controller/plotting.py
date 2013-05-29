# Python imports
import sys
import math

# PySide imports
from PySide import QtGui, QtCore

# MSL Sim imports
from sim.model.lines import get_line_dict
import sim.defaults as d


class PlotGraphicsView(QtGui.QGraphicsView):
    def __init__(self, parent):
        super(PlotGraphicsView, self).__init__(parent)
        self.line_map = []
        self.scale(20, 20)
        self.scan_list = []
        self.set_colours()
        self.g_scene = QtGui.QGraphicsScene(self)
        self.setScene(self.g_scene)
        self.setSceneRect(0, 0, d.MAP_WIDTH, d.MAP_HEIGHT)

    def initialiseRobot(self):
        """Draws the robot in the scene."""
        self.rect = self.scene().addRect(self.robot.x - self.robot.length/2.0,
                                         self.robot.y - self.robot.width/2.0,
                                         self.robot.length, self.robot.width,
                                         self.robot_pen)
        self.rect.setTransformOriginPoint(self.robot.x,
                                          self.robot.y)

    def start_timers(self):
        """Starts separate timers to update the plot, odometry, and range data
        from the laser."""
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.plot_update)
        self.plot_timer.start(1000/d.PLOT_FREQ)
        self.odom_timer = QtCore.QTimer()
        self.odom_timer.timeout.connect(self.robot.update_pose)
        self.odom_timer.start(1000/d.ODOM_FREQ)
        self.laser_timer = QtCore.QTimer()
        self.laser_timer.timeout.connect(
                lambda: self.robot.scan_laser(self.line_map))
        self.laser_timer.start(1000/d.LASER_FREQ)

    def set_colours(self):
        """Creates QPen instances for all the objects that will be plotted."""
        self.laser_pen = QtGui.QPen()
        self.robot_pen = QtGui.QPen()
        red = QtGui.QColor(255, 0, 0)
        green = QtGui.QColor(0, 255, 0)
        red.setAlpha(40)
        self.laser_pen.setColor(red)
        self.robot_pen.setColor(green)

    def plot_update(self):
        """Updates the plot. This method is called automatically by the
        plot_timer."""
        if self.robot.moved:
            self.rect.setX(self.robot.x)
            self.rect.setY(self.robot.y)
            self.rect.setRotation(180/math.pi * self.robot.heading)
            self.robot.moved = False
        if self.robot.sized:
            self.rect.setWidth(self.robot.width)
            self.rect.setHeight(self.robot.height)
            self.robot.sized = False
        if self.robot.scanned:
            self.draw_laser_beams(self.robot.scan_history[-1])
            self.robot.scanned = False

    def draw_laser_beams(self, scan):
        """Deletes any previously drawn laser beams and plots the latest laser
        measurement."""
        pose, ranges = scan
        x, y, theta = self.robot.pose
        laser_range = self.robot.laser.range
        laser_min_angle = self.robot.laser.min_angle
        laser_res = self.robot.laser.resolution
        # Empty out the scan group
        if self.scan_list:
            for item in self.scan_list:
                self.scene().removeItem(item)
            self.scan_list = []
        for (i, r) in enumerate(ranges):
            if r == 0:
                r = laser_range
            x_2 = x + math.pi/180 + r*math.cos(
                    theta + math.pi/180*(laser_min_angle + laser_res*i))
            y_2 = y + math.pi/180 + r*math.sin(
                    theta + math.pi/180*(laser_min_angle + laser_res*i))
            self.scan_list.append(
                    self.scene().addLine(QtCore.QLineF(x, y, x_2, y_2), self.laser_pen))

    def keyPressEvent(self, event):
        """Adjusts the translational and angular velocites of the robot."""
        if event.key() == QtCore.Qt.Key_W:
            self.robot.vel += 0.1
        elif event.key() == QtCore.Qt.Key_S:
            self.robot.vel -= 0.1
        elif event.key() == QtCore.Qt.Key_A:
            self.robot.ang_vel -=  math.pi/24
        elif event.key() == QtCore.Qt.Key_D:
            self.robot.ang_vel +=  math.pi/24
        if event.key() == QtCore.Qt.Key_Escape:
            sys.exit()

    def mousePressEvent(self, event):
        """Records the coordinates of the point where the mouse was clicked on
        the scene."""
        if event.button() == QtCore.Qt.LeftButton:
            self.start = QtCore.QPointF(self.mapToScene(event.pos()))

    def mouseReleaseEvent(self, event):
        """Draws a line from the recorded click coordinates and the position of 
        the cursor when the left mouse button is released. Adds this line to
        the line map."""
        if event.button() == QtCore.Qt.LeftButton:
            end = QtCore.QPointF(self.mapToScene(event.pos()))
            line = get_line_dict(self.start.x(), self.start.y(),
                    end.x(), end.y())
            self.line_map.append(line)
            self.scene().addItem(
                    QtGui.QGraphicsLineItem(QtCore.QLineF(self.start, end)))
