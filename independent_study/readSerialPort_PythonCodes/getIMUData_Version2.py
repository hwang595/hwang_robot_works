import serial

ser = serial.Serial('COM6', 9600)

print(ser.is_open)

ser.flushInput()
ser.flushOutput()
accel = []
gyro = []
quaternion = []
#magnetic = []

readData = ser.readline()
lineContent = re.search('b\'(.+)\'', str(readData)).group(1)
splitData = re.search('accelData: (.+) gyroData: (.+) quaternion: (.+)',\
                      lineContent)
print(splitData)

splitDataTmp = splitData.group(1)
accelDataTmp = splitData[0]
gyroDataTmp = splitData[1]
qDataTmp = splitData[2]

accel = re.search('ax=(.+), ay=(.+), az=(.+)', accelDataTmp).group(1)
gyro = re.search('gx=(.+), gy=(.+), gz=(.+)', gyroDataTmp).group(1)
quaternion = re.search('q0=(.+), q1=(.+), q2=(.+), q3=(.+)', qDataTmp).group(1)
