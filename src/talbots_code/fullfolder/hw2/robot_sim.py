import redis
import matplotlib.pyplot as plt
import numpy as np
import string
import sys

# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import *

class redisGUI(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()
        self.initUI(500)

    def initUI(self, updateTime):
        # Update time of graph in ms
        self.updateTime = updateTime

        ##### Formatting #####
        # Sets up labels for GUI
        robotLabel = QLabel("Robot:")
        qLabel = QLabel("q:")
        dqLabel = QLabel("dq:")
        fgcLabel = QLabel("fgc:")
        plotInfoLabel = QLabel("Choose Info to Display:")

        # Sets up information labels
        self.q = QLabel()
        self.dq = QLabel()
        self.fgc = QLabel()

        # Sets up redis server to allow reading data
        self.r = redis.StrictRedis(host='localhost', port=6379, db=0)

        ##### Information Plotting #####
        # Configures combo box of all robots in list
        robots = QComboBox()
        self.fillRobots(robots)
        robots.activated[str].connect(self.onRobotChange)
        # Configures combo box of different information to plot
        self.plotInfo = QComboBox()
        self.plotInfo.activated[str].connect(self.onInfoChange)

        # Plotting backend
        self.figure = plt.figure()
        plt.ion()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        # Sets up timer to update requested variables periodically
        timer=QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(self.updateTime)

        ##### Layout #####
        grid = QGridLayout()
        grid.setSpacing(5)

        # Adds list of all robots to be chosen to display
        grid.addWidget(robotLabel, 1, 0)
        grid.addWidget(robots, 1, 1)
        # Adds widgets corresponding to common display variables
        grid.addWidget(qLabel, 2, 0)
        grid.addWidget(self.q, 2, 1)
        grid.addWidget(dqLabel, 3, 0)
        grid.addWidget(self.dq, 3, 1)
        grid.addWidget(fgcLabel, 4, 0)
        grid.addWidget(self.fgc, 4, 1)
        # Adds list of all information that can be chosen to display
        grid.addWidget(plotInfoLabel, 5, 0)
        grid.addWidget(self.plotInfo, 5, 1)

        # Adds Matplotlib to display
        grid.addWidget(self.toolbar, 6, 1)
        grid.addWidget(self.canvas, 7, 1, 2, 2)
        
        self.setLayout(grid) # Sets layout of widget to be grid layout
        self.resize(640, 640)
        self.move(240, 240)
        self.setWindowTitle("Redis GUI")
        self.show()

    # When robot chosen, updates fields to show values of new robot
    def onRobotChange(self, text):
        if text == "Choose Robot":
            return
        
        # Plots q by default so plotting doesn't crash on startup
        self.robot = "scl::robot::" + text + "::"
        self.key = "Choose Info"
        
        self.fillPlotInfo()
        self.clearPlot()
        self.update()

    # When robot info chosen, sets key to plot requested info
    def onInfoChange(self, text):
        if text == "Choose Info":
            return
        self.key = text
        self.clearPlot()

    # Data needed for clean instance of plotting when switching
    def clearPlot(self):
        if self.key == "Choose Info":
            return
        plt.clf()
        self.figure.suptitle(self.key, fontsize=16)
        plt.xlabel('Time (seconds)')
        plt.ylabel('Value (SI units)')
        self.numb = len(string.split(self.r.get(self.key)))
        self.Dat = np.array([[] for i in range(self.numb)])
        self.Tim = []
        self.t = 0
        
    # Updates all current fields with values from redis server
    def update(self):
        if self.robot == "Choose Robot" or self.key == "Choose Info":
            return
        
        self.r.get("x_pos")
        self.q.setText(self.r.get("x_pos"))

        data = np.array([[float(e)] for e in string.split(self.r.get(self.key))])
        self.Dat = np.hstack((self.Dat, data))
        self.Tim.append(self.t)
        
        # Plot each component of all data (typically by joint)
        for i in range(self.numb):
            plt.plot(self.Tim, self.Dat[i], hold=True, label=str(i))

        if self.t == 0:
            plt.legend()
            
        plt.xlim(max(0, self.t - 10), self.t)
        self.t = self.t + self.updateTime/1000.0

    # Reads in robots from redis server and adds choices to list
    def fillRobots(self, robots):
        self.robot = "Choose Robot"
        robots.clear()
        robots.addItem("Choose Robot")
        allRobots = self.r.smembers("scl::robots")
        for robot in allRobots:
            robots.addItem(robot)

    # Reads in information about selected robot and adds to choices
    def fillPlotInfo(self):
        self.key = "Choose Info"
        self.plotInfo.clear()
        self.plotInfo.addItem("Choose Info")
        for info in self.r.keys():
            if self.robot in info:
                self.plotInfo.addItem(info)
            
a = QApplication(sys.argv)
window = redisGUI()
sys.exit(a.exec_())