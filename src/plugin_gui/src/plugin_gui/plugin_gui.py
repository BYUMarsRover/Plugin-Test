#!/usr/bin/python

# Ryan Black
# Mars Rover 2022
# Plugin prototype GUI

import sys
print(sys.version)
import warnings
warnings.filterwarnings('ignore')


from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QComboBox
from PyQt5.QtCore import QSize, Qt, pyqtSlot,pyqtSignal
from rover_plugins.plugin_base import Plugin_Base

# Register plugin types here
from rover_plugins.waypoint_plugin import Waypoint_Plugin


class HelloWindow(QMainWindow):
    pushButtonClicked = pyqtSignal()
    
    def pushButtonClicked(self):
        self.title.setText(self.pluginSelector.currentText() + " selected!")
    
    def __init__(self):
        QMainWindow.__init__(self)
 
        self.setMinimumSize(QSize(640, 480))    
        self.setWindowTitle("Hello world") 
 
        self.title = QLabel("PyQt!", self)
        self.title.setAlignment(QtCore.Qt.AlignCenter)

        self.pluginSelector = QtWidgets.QComboBox(self)
        self.pluginSelector.setGeometry(QtCore.QRect(100, 50, 100, 30))
        for plugin in Plugin_Base.__subclasses__():
            self.pluginSelector.addItem(plugin.__name__)
        self.pluginSelector.currentIndexChanged.connect(self.pushButtonClicked)
 
        self.pushButton = QtWidgets.QPushButton(self)
        self.pushButton.setGeometry(QtCore.QRect(140, 200, 99, 27))#x,y, width, height
        self.pushButton.setObjectName("pushButton")
        self.pushButton.setText("Click Me!")
        self.pushButton.clicked.connect(self.pushButtonClicked)      
 
 
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = HelloWindow()
    mainWin.show()
    sys.exit( app.exec_() )
