# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sim/view/plotting.ui'
#
# Created: Thu May 23 10:20:12 2013
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_plot(object):
    def setupUi(self, plot):
        plot.setObjectName("plot")
        plot.resize(442, 332)
        self.horizontalLayout = QtGui.QHBoxLayout(plot)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.graphics_view = PlotGraphicsView(plot)
        self.graphics_view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.graphics_view.setSceneRect(QtCore.QRectF(0.0, 0.0, 10.0, 10.0))
        self.graphics_view.setObjectName("graphics_view")
        self.horizontalLayout.addWidget(self.graphics_view)

        self.retranslateUi(plot)
        QtCore.QMetaObject.connectSlotsByName(plot)

    def retranslateUi(self, plot):
        plot.setWindowTitle(QtGui.QApplication.translate("plot", "Form", None, QtGui.QApplication.UnicodeUTF8))

from sim.controller.plotting import PlotGraphicsView
