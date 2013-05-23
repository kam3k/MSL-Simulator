# Python imports
import sys

# PySide imports
from PySide import QtGui

# MSL Sim imports
from sim.controller.plotting import PlotWidget


app = QtGui.QApplication(sys.argv)
plot_widget = PlotWidget()
plot_widget.show()
sys.exit(app.exec_())
