# Python imports
import sys

# PySide imports
from PySide import QtGui

# MSL Sim imports
from sim.controller.window import MainWindow


app = QtGui.QApplication(sys.argv)
main_window = MainWindow()
main_window.show()
sys.exit(app.exec_())
