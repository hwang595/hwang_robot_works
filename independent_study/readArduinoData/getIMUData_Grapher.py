#code -*- -utf8
import serial
import re
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import time
#created on 10/31/2016
__author__ = 'hwang'
__version__ = '1.0.0'

#Note that, here we define the format of serial from the Arduino is:
#'yaw,pitch,roll,gx,gy,gz,ax,ay,az,mx,my,mz,g0,g1,g2,g3,force,angle'

class IMU_data:
    '''
    this class gives baisc data from IMU module, which include data from gyroscope
    accelerations, and quaternions
    '''
    def __init__(self, serPort='COM6', baudRate=115200):
        self.ser = serial.Serial(serPort, baudRate)
        self.dataLine = []
        self.euAngle = [] #contains Yaw, Pitch, Roll
        self.gyro = []  # represent the data from gyroscope
        self.accel = [] #represent the accelation data
        self.magnet = [] #strength of magnetic field
        self.quaternion = [] #represent of the quaternion data
        self.force = 0 #represent the force data from the force sensor
        self.angle = 0 #represent the angle between the tongs
        self.raw = ''

    def dataUpdate(self):
        #do not flush the I/O region of the serial port, that's important

        readData = self.ser.readline()
        readData = str(readData).strip(' ').strip('b').strip('\'').strip('\\r\\n')
        data_match = re.match('(.*?)\$(.+)', readData)
        self.raw = readData

        if data_match != None:
            data_search = data_match.group(2)
        else:
            print('Data format Error!')
            return

        self.dataLine = data_search.split(',')
        idx = 0
        self.euAngle = self.dataLine[idx:idx+3]
        idx += 3
        self.gyro = self.dataLine[idx:idx+3]
        idx += 3
        self.accel = self.dataLine[idx:idx + 3]
        idx += 3
        self.magnet = self.dataLine[idx:idx + 3]
        idx += 3
        self.quaternion = self.dataLine[idx:idx + 4]
        idx += 4
        self.force = self.dataLine[idx:idx + 1]
        idx += 1
        self.angle = self.dataLine[idx:idx + 1]

    def logDataToFile(self, fileName='logData.txt'):
        timeStamp = time.strftime('%a %H:%M:%S')
        timeStamp = timeStamp[-8:]
        writeTmp = self.raw[1:]
        writeContent = timeStamp + ',' + writeTmp
        with open(fileName, 'a') as writeData:
            writeData.writelines(writeContent)
            writeData.write('\r')


app = QtGui.QApplication([])

win = pg.GraphicsWindow(title='Real Time Plotting Demo, Author: HWang')
win.resize(1200, 600)
pg.setConfigOptions(antialias=True)
force = win.addPlot(title="Force Data")
accel = win.addPlot(title="Accelerations")
gyro = win.addPlot(title='Gyroscope')
angle = win.addPlot(title='Angle')

imu_data = IMU_data()
force.setXRange(0, 550)
force.setYRange(0, 8)
accel.setXRange(0, 550)
accel.setYRange(-1.5, 1.5)
gyro.setXRange(0, 550)
gyro.setYRange(-3000, 3000)
angle.setXRange(0, 550)
angle.setYRange(0, 45)

curveForce = force.plot(pen='b')
curveAngle = angle.plot(pen='b')
curveAX = accel.plot(pen='r')
curveAY = accel.plot(pen='g')
curveAZ = accel.plot(pen='b')

curveGX = gyro.plot(pen='r')
curveGY = gyro.plot(pen='g')
curveGZ = gyro.plot(pen='b')

forceData = np.array([])
angleData = np.array([])
AX = np.array([])
AY = np.array([])
AZ = np.array([])
GX = np.array([])
GY = np.array([])
GZ = np.array([])
cur_length = 0

ptr = 0
i = 0

def plot_update(): #update the plotting data in real time, the length of the data sequence is 500
    global curveForce, forceData, curveAngle, angleData,\
        AX, AY, AZ, curveAX, curveAY, curveAZ, \
        GX, GY, GZ, curveGX, curveGY, curveGZ, \
        ptr, force, accel, gyro, imu_data, cur_length, angle

    cur_force = float(imu_data.force[0])
    cur_angle = float(imu_data.angle[0])
    accelX = float(imu_data.accel[0])
    accelY = float(imu_data.accel[1])
    accelZ = float(imu_data.accel[2])
    gyroX = float(imu_data.gyro[0])
    gyroY = float(imu_data.gyro[1])
    gyroZ = float(imu_data.gyro[2])

    if cur_length >= 500:
        forceData = np.delete(forceData, 0)
        angleData = np.delete(angleData, 0)
        AX = np.delete(AX, 0)
        AY = np.delete(AY, 0)
        AZ = np.delete(AZ, 0)
        GX = np.delete(GX, 0)
        GY = np.delete(GY, 0)
        GZ = np.delete(GZ, 0)

    forceData = np.append(forceData, cur_force)
    angleData = np.append(angleData, cur_angle)
    cur_length = len(forceData)
    AX = np.append(AX, accelX)
    AY = np.append(AY, accelY)
    AZ = np.append(AZ, accelZ)
    GX = np.append(GX, gyroX)
    GY = np.append(GY, gyroY)
    GZ = np.append(GZ, gyroZ)

    curveForce.setData(forceData)
    curveAngle.setData(angleData)
    curveAX.setData(AX)
    curveAY.setData(AY)
    curveAZ.setData(AZ)
    curveGX.setData(GX)
    curveGY.setData(GY)
    curveGZ.setData(GZ)
#    print("%.2f, %.2f, %.2f" % (accelX, accelY, accelZ))
    print("%.2f, %.2f, %.2f" % (gyroX, gyroY, gyroZ))

    if ptr == 0:
        force.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr = 1

#using the QtTimer to do something like a loop here
timer = QtCore.QTimer()
timer.timeout.connect(imu_data.dataUpdate)
timer.timeout.connect(plot_update)
timer.start(0)

if __name__ == '__main__':
    import sys
    # create a QtApp property for this program, this is very important
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
