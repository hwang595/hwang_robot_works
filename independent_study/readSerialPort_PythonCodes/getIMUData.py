#demo of getIMUData
#Version 1.0
#Author: HWang
#There is a status machine in this code to help match the data

import serial
import sys
import re

#pNum = sys.argv[1]
#pBdRate = sys.argv[2]

ser = serial.Serial('COM6', 9600)

print(ser.is_open)
ser.flushInput()
ser.flushOutput()
accel = []
gyro = []
quaternion = []
#magnetic = []

statusMachineLabel = 0 #in which 0 means the original status

while True:
#    bytesToRead = ser.inWaiting()
#    readData = ser.read(bytesToRead)
    readData = ser.readline()
    lineContent = re.search('b\'(.+)\'', str(readData)).group(1)
    lineContent = lineContent[:-4]
    if lineContent == 'accelData:' and statusMachineLabel == 0:
        statusMachineLabel = 1
    if statusMachineLabel == 1 and \
       re.search('ax=(.+), ay=(.+), az=(.+)', lineContent) != None:
        accelTmp = re.search('ax=(.+), ay=(.+), az=(.+)', lineContent)
        for i in range(1, 4):
            accel.append(float(accelTmp.group(i)))
        statusMachineLabel = 2 #2 means we got the accel data and go ahead to find gyro
        
    if lineContent == 'gyroData:' and statusMachineLabel == 2:
        statusMachineLabel = 3
    if statusMachineLabel == 3 and \
       re.search('gx=(.+), gy=(.+), gz=(.+)', lineContent) != None:
        gyroTmp = re.search('gx=(.+), gy=(.+), gz=(.+)', lineContent)
        for i in range(1, 4):
            gyro.append(float(gyroTmp.group(i)))
        statusMachineLabel = 4
        
    if lineContent == 'quaternion:' and statusMachineLabel == 4:
        statusMachineLabel = 5
    if statusMachineLabel == 5 and \
       re.search('q0=(.+), q1=(.+), q2=(.+), q3=(.+)', lineContent) != None:    
        q = re.search('q0=(.+), q1=(.+), q2=(.+), q3=(.+)', lineContent)
        for i in range(1, 5):
            quaternion.append(float(q.group(i)))
        statusMachineLabel = 0
    if(len(accel)):
        print('Acceleration data:')
        for i in range(len(accel)):
            print(accel[i], end=' ')
        print('')
        print('Gyro data:')
    if(len(gyro)):
        for i in range(len(gyro)):
            print(gyro[i], end=' ')
        print('')
    if(len(quaternion)):
        print('Quaternion:')
        for i in range(len(quaternion)):
            print(quaternion[i], end=' ')
        print('')
    accel = []
    gyro = []
    quaternion = []
