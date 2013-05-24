# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sim/view/window.ui'
#
# Created: Thu May 23 22:38:10 2013
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_main_window(object):
    def setupUi(self, main_window):
        main_window.setObjectName("main_window")
        main_window.resize(1404, 767)
        main_window.setMinimumSize(QtCore.QSize(0, 0))
        self.centralwidget = QtGui.QWidget(main_window)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.plot_widget = PlotWidget(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_widget.sizePolicy().hasHeightForWidth())
        self.plot_widget.setSizePolicy(sizePolicy)
        self.plot_widget.setMinimumSize(QtCore.QSize(500, 500))
        self.plot_widget.setObjectName("plot_widget")
        self.gridLayout.addWidget(self.plot_widget, 0, 0, 3, 1)
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(2)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tabWidget.setTabShape(QtGui.QTabWidget.Rounded)
        self.tabWidget.setUsesScrollButtons(False)
        self.tabWidget.setObjectName("tabWidget")
        self.tabWidgetPage2 = QtGui.QWidget()
        self.tabWidgetPage2.setObjectName("tabWidgetPage2")
        self.tabWidget.addTab(self.tabWidgetPage2, "")
        self.tab_3 = QtGui.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.tabWidget.addTab(self.tab_3, "")
        self.settings = QtGui.QWidget()
        self.settings.setObjectName("settings")
        self.gridLayout_4 = QtGui.QGridLayout(self.settings)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.laser_widget = LaserWidget(self.settings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.laser_widget.sizePolicy().hasHeightForWidth())
        self.laser_widget.setSizePolicy(sizePolicy)
        self.laser_widget.setMinimumSize(QtCore.QSize(400, 0))
        self.laser_widget.setMaximumSize(QtCore.QSize(400, 16777215))
        self.laser_widget.setObjectName("laser_widget")
        self.gridLayout_4.addWidget(self.laser_widget, 3, 0, 1, 1)
        self.robot_widget = RobotWidget(self.settings)
        self.robot_widget.setMinimumSize(QtCore.QSize(400, 0))
        self.robot_widget.setMaximumSize(QtCore.QSize(400, 16777215))
        self.robot_widget.setObjectName("robot_widget")
        self.gridLayout_4.addWidget(self.robot_widget, 1, 0, 1, 1)
        self.widget = OdometryWidget(self.settings)
        self.widget.setMinimumSize(QtCore.QSize(400, 0))
        self.widget.setMaximumSize(QtCore.QSize(400, 16777215))
        self.widget.setObjectName("widget")
        self.gridLayout_4.addWidget(self.widget, 2, 0, 1, 1)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.load_settings_button = QtGui.QPushButton(self.settings)
        self.load_settings_button.setObjectName("load_settings_button")
        self.horizontalLayout.addWidget(self.load_settings_button)
        self.save_settings_button = QtGui.QPushButton(self.settings)
        self.save_settings_button.setObjectName("save_settings_button")
        self.horizontalLayout.addWidget(self.save_settings_button)
        self.gridLayout_4.addLayout(self.horizontalLayout, 4, 0, 1, 1)
        self.tabWidget.addTab(self.settings, "")
        self.gridLayout.addWidget(self.tabWidget, 0, 1, 3, 1)
        main_window.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar()
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1404, 22))
        self.menubar.setObjectName("menubar")
        main_window.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(main_window)
        self.statusbar.setObjectName("statusbar")
        main_window.setStatusBar(self.statusbar)

        self.retranslateUi(main_window)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(main_window)

    def retranslateUi(self, main_window):
        main_window.setWindowTitle(QtGui.QApplication.translate("main_window", "MSL Sim", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabWidgetPage2), QtGui.QApplication.translate("main_window", "Map", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), QtGui.QApplication.translate("main_window", "Recording", None, QtGui.QApplication.UnicodeUTF8))
        self.load_settings_button.setText(QtGui.QApplication.translate("main_window", "Load settings...", None, QtGui.QApplication.UnicodeUTF8))
        self.save_settings_button.setText(QtGui.QApplication.translate("main_window", "Save settings...", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.settings), QtGui.QApplication.translate("main_window", "Settings", None, QtGui.QApplication.UnicodeUTF8))

from sim.controller.plotting import PlotWidget
from sim.controller.odometry import OdometryWidget
from sim.controller.laser import LaserWidget
from sim.controller.robot import RobotWidget
