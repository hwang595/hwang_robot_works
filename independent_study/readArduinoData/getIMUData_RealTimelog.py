#code -*- -utf8
import serial
import re
import numpy as np
import time
#created on 10/31/2016
__author__ = 'hwang'
__version__ = '1.0.0'

#Note that, here we define the format of serial from the Arduino is:
#'yaw,pitch,roll,gx,gy,gz,ax,ay,az,mx,my,mz,g0,g1,g2,g3,force,angle'

class Arduino_data:
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
        global last_timeStamp, i
        timeStamp = time.strftime('%a %H:%M:%S')
        timeStamp = timeStamp[-8:]
        cur_timeStamp = timeStamp
        if cur_timeStamp != last_timeStamp:
            i = 0
        i += 1
        last_timeStamp = cur_timeStamp
        writeTmp = self.raw[1:]
        writeContent = str(i) + ' ' + timeStamp + ':' + writeTmp
        with open(fileName, 'a') as writeData:
            writeData.writelines(writeContent)
            writeData.write('\r')

arduino_data = Arduino_data()
stamp_value = 0
last_timeStamp = ''

if __name__ == '__main__':
   while True:
       arduino_data.dataUpdate()
       arduino_data.logDataToFile()
       print('Done')

