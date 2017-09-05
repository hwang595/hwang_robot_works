#code -*- -utf8
import serial
import re
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import time
#created on 11/15/2016
__author__ = 'hwang'
__version__ = '1.0.0'

#Note that, here we define the format of serial from the Arduino is:
#'$, force_channel0, force_channle1'

class Force_data:
    '''
    this class gives baisc data from IMU module, which include data from gyroscope
    accelerations, and quaternions
    '''
    def __init__(self, serPort='COM3', baudRate=9600):
        self.ser = serial.Serial(serPort, baudRate)
        self.dataLine = []
        self.force_channel_0 = 0
        self.force_channel_1 = 0

    def dataUpdate(self):
        #do not flush the I/O region of the serial port, that's important

        readData = self.ser.readline()
        readData = str(readData).strip(' ').strip('b').strip('\'').strip('\\r\\n')
        data_match = re.match('(.*?)\$(.+)', readData)

        if data_match != None:
            data_search = data_match.group(2)
        else:
            print('Data format Error!')
            return

        self.dataLine = data_search.split(',')
        idx = 1
        self.force_channel_0 = float(self.dataLine[idx])
        idx += 1
        self.force_channel_1 = float(self.dataLine[idx])

force_data = Force_data()

def deleteContent(fileName):
    with open(fileName, 'w'):
        pass

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title='Real Time Plotting Demo, Author: HWang')
win.resize(1200, 600)
pg.setConfigOptions(antialias=True)
force0 = win.addPlot(title="Force Data Channel0")
force1 = win.addPlot(title="Force Data Channel1")

force0.setXRange(0, 550)
force0.setYRange(0, 144)
force1.setXRange(0, 550)
force1.setYRange(0, 144)

curveForce0 = force0.plot(pen='r')
curveForce1 = force1.plot(pen='g')

forceData0 = np.array([])
forceData1 = np.array([])
cur_length = 0

ptr = 0
i = 0

def plot_update(): #update the plotting data in real time, the length of the data sequence is 500
    global  froce0, force1, curveForce0, curveForce1,\
            forceData0, forceData1,\
        ptr, i, force_data, cur_length

    cur_force0 = force_data.force_channel_0
    cur_force1 = force_data.force_channel_1

    if cur_length >= 500:
        forceData0 = np.delete(forceData0, 0)
        forceData1 = np.delete(forceData1, 0)

    forceData0 = np.append(forceData0, cur_force0)
    cur_length = len(forceData0)
    forceData1 = np.append(forceData1, cur_force1)

    curveForce0.setData(forceData0)
    curveForce1.setData(forceData1)

    if ptr == 0:
        force0.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
        force1.enableAutoRange('xy', False)
    ptr = 1

#using the QtTimer to do something like a loop here
timer = QtCore.QTimer()
timer.timeout.connect(force_data.dataUpdate)
timer.timeout.connect(plot_update)
timer.start(0)

if __name__ == '__main__':
    import sys
    # create a QtApp property for this program, this is very important
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
