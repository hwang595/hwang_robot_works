# -*- coding: utf-8 -*-
#created on 10/28/2016
#update on 10/31/2016

__author__ = 'drakita & hwang'
__version__ = '2.0.0'

'''
This code gives solution to plot the tcp_forces of the UR5 robot in real time
=============================================================================
Updata Info: add subplot to show rx, ry, ry for tcp_force
             add subplot to show q_actual data
             add subplot to show qd_actual data
'''

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import socket
from struct import *

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title='Real Time Plotting by drakita & hwang')
pg.setConfigOptions(antialias=True)
win.resize(2000, 700)

jPos_1 = win.addPlot(title='Joint Position Dim1')
jPos_2 = win.addPlot(title='Joint Position Dim2')
jPos_3 = win.addPlot(title='Joint Position Dim3')
jPos_4 = win.addPlot(title='Joint Position Dim4')
jPos_5 = win.addPlot(title='Joint Position Dim5')
jPos_6 = win.addPlot(title='Joint Position Dim6')
win.nextRow()

jV_1 = win.addPlot(title='Joint Velocity Dim1')
jV_2 = win.addPlot(title='Joint Velocity Dim2')
jV_3 = win.addPlot(title='Joint Velocity Dim3')
jV_4 = win.addPlot(title='Joint Velocity Dim4')
jV_5 = win.addPlot(title='Joint Velocity Dim5')
jV_6 = win.addPlot(title='Joint Velocity Dim6')
win.nextRow()
c = win.addPlot(title="Plot tcp_force: x,y,z")
d = win.addPlot(title="Plot tcp_force: rx,ry,rz")

class UR5_dataReader:
    '''
    This class allows for 125Hz reading of all data from the ur5.
    First call update() to update data, then access data
    '''

    def __init__(self, sock=None, ip="192.168.1.102"):
        '''
        constructor
        :param socket: can optionally pass in a socket if this class is used in a larger framework
        :param ip: ip address of robot
        :return:
        '''
        if not sock == None:
            self.sock = sock
        else:
            self.HOST = ip
            self.PORT = 30003
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.HOST,self.PORT))
            self.sock.settimeout(2) # very important!
            # print self.sock.recv(1024)
            # self.outFile = open('/home/drakita/Desktop/force_output', 'w')

        #  variables
        self.line = ''
        self.messageSize = 1060
        self.time = 0.0 # time elapsed since the controller was started
        self.q_target = 6*[0.0] # target joint positions
        self.qd_target = 6*[0.0] # target joint velocities
        self.qdd_target = 6*[0.0] # target joint accelerations
        self.i_target = 6*[0.0] # target joint currents
        self.m_target = 6*[0.0] # target torques
        self.q_actual = 6*[0.0] # actual joint positions
        self.qd_actual = 6*[0.0] # actual joint velocities
        self.i_actual = 6*[0.0] # actual joint currents
        self.i_control = 6*[0.0] # joint control currents
        self.tool_vec_actual = 6*[0.0] # actual cartesian coordinates of the tool (x,y,z,rx,ry,rz)
        self.tcp_speed_actual = 6*[0.0] # actual speed of the tool given in cartesian coords
        self.tcp_force = 6*[0.0] # genearlized forces in the tcp
        self.tool_vec_target = 6*[0.0] # target cartesian coordinates of the tool (x,y,z,rx,ry,rz)
        self.tcp_speed_target = 6*[0.0] # target speed of the tool given in cartesian coords
        self.digital_input_bits = 0.0 # crrent state of the digital inputs
        self.motor_temps = 6*[0.0] # motor temperatures (celsius)
        self.controller_timer = 0.0 # controller realtime thread execution time
        self.test_value = 0.0 # a value used by UR software only
        self.robot_mode = 0.0 # robot mode
        self.joint_modes = 6*[0.0] # joint control modes
        self.safety_mode = 0.0
        self.tool_accel_values = 3*[0.0] # tool accelerometer values
        self.speed_scaling = 0.0 # speed scaling of the trajectory limiter
        self.lin_momentum_norm = 0.0 # norm of cartesian linear momentum
        self.v_main = 0.0 # masterboard: main voltage
        self.v_robot = 0.0 #masterboard:robot voltage (48v)
        self.i_robot = 0.0 # materboard: robot current
        self.v_actual = 6*[0.0] # actual joint voltages
        self.digital_outputs = 0.0 # digitial outputs
        self.program_state = 0.0 # program state

    def robot_update(self):
        recv = None
        try:
            recv = self.sock.recv(1060)
        except Exception:
            pass

        if recv == None:
            return

        if not len(recv) == 1060:
            self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.HOST,self.PORT))
            self.sock.settimeout(0.5) # very important!
            return

        format = "!idddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"

        line = unpack(format, recv)

        self.line = line
        idx = 0
        self.messageSize = line[idx]
        idx += 1
        self.time = line[idx]
        idx += 1
        self.q_target = line[idx:idx+6]
        idx += 6
        self.qd_target = line[idx:idx+6]
        idx += 6
        self.qdd_target = line[idx:idx+6]
        idx += 6
        self.i_target = line[idx:idx+6]
        idx += 6
        self.m_target = line[idx:idx+6]
        idx += 6
        self.q_actual = line[idx:idx+6]
        idx += 6
        self.qd_actual = line[idx:idx+6]
        idx += 6
        self.i_actual = line[idx:idx+6]
        idx += 6
        self.i_control = line[idx:idx+6]
        idx += 6
        self.tool_vec_actual = line[idx:idx+6]
        idx += 6
        self.tcp_speed_actual = line[idx:idx+6]
        idx += 6
        self.tcp_force = line[idx:idx+6]
        idx += 6
        self.tool_vec_target = line[idx:idx+6]
        idx += 6
        self.tcp_speed_target = line[idx:idx+6]
        idx += 6
        self.digital_input_bits = line[idx]
        idx += 1
        self.motor_temps = line[idx:idx+6]
        idx += 6
        self.controller_timer = line[idx]
        idx += 1
        self.test_value = line[idx]
        idx += 1
        self.robot_mode = line[idx]
        idx += 1
        self.joint_modes = line[idx:idx+6]
        idx += 6
        self.safety_mode = line[idx]
        idx += 1
        idx += 6 # empty
        self.tool_accel_values = line[idx:idx+3]
        idx += 3
        idx += 6 # empty
        self.speed_scaling = line[idx]
        idx += 1
        self.lin_momentum_norm = line[idx]
        idx += 1
        idx += 2 # empty
        self.v_main = line[idx]
        idx += 1
        self.v_robot = line[idx]
        idx += 1
        self.i_robot = line[idx]
        idx += 1
        self.v_actual = line[idx:idx+6]

dr = UR5_dataReader()

#set range of subplot for the first row
jPos_1.setXRange(0, 550)
jPos_1.setYRange(-4, 4)
jPos_2.setXRange(0, 550)
jPos_2.setYRange(-4, 4)
jPos_3.setXRange(0, 550)
jPos_3.setYRange(-4, 4)
jPos_4.setXRange(0, 550)
jPos_4.setYRange(-4, 4)
jPos_5.setXRange(0, 550)
jPos_5.setYRange(-4, 4)
jPos_6.setXRange(0, 550)
jPos_6.setYRange(-4, 4)

#set range of subplot for the second row
jV_1.setXRange(0, 550)
jV_1.setYRange(-0.3, 0.3)
jV_2.setXRange(0, 550)
jV_2.setYRange(-0.3, 0.3)
jV_3.setXRange(0, 550)
jV_3.setYRange(-0.3, 0.3)
jV_4.setXRange(0, 550)
jV_4.setYRange(-0.3, 0.3)
jV_5.setXRange(0, 550)
jV_5.setYRange(-0.3, 0.3)
jV_6.setXRange(0, 550)
jV_6.setYRange(-0.3, 0.3)

#set range of subplot for tcp_forces
c.setXRange(0, 550)
c.setYRange(-150, 150)
d.setXRange(0, 550)
d.setYRange(-30, 30)

#subplot setting for q_actual: dim 1 ~ 6
jPos_1_curve = jPos_1.plot(pen='r')
jPos_2_curve = jPos_2.plot(pen='g')
jPos_3_curve = jPos_3.plot(pen='b')
jPos_4_curve = jPos_4.plot(pen='y')
jPos_5_curve = jPos_5.plot(pen=(255, 255, 255))
jPos_6_curve = jPos_6.plot(pen=(0, 255, 255))
position_1 = np.array([])
position_2 = np.array([])
position_3 = np.array([])
position_4 = np.array([])
position_5 = np.array([])
position_6 = np.array([])

#subplot setting for qd_actual: dim 1 ~ 6
jV1_curve = jV_1.plot(pen='r')
jV2_curve = jV_2.plot(pen='g')
jV3_curve = jV_3.plot(pen='b')
jV4_curve = jV_4.plot(pen='y')
jV5_curve = jV_5.plot(pen=(255, 255, 255))
jV6_curve = jV_6.plot(pen=(0, 255, 255))
velocity_1 = np.array([])
velocity_2 = np.array([])
velocity_3 = np.array([])
velocity_4 = np.array([])
velocity_5 = np.array([])
velocity_6 = np.array([])

#subplot setting for tcp_force: x,y,z
force_curveX = c.plot(pen='r')
force_curveY = c.plot(pen='g')
force_curveZ = c.plot(pen='b')
forceX = np.array([])
forceY = np.array([])
forceZ = np.array([])

#subplot setting for tcp_force: rx,ry,rz
force_curveRX = d.plot(pen='r')
force_curveRY = d.plot(pen='g')
force_curveRZ = d.plot(pen='b')
forceRX = np.array([])
forceRY = np.array([])
forceRZ = np.array([])

cur_length = 0

ptr = 0
i = 0


def plot_update(): #update the plotting data in real time, the length of the data sequence is 500
    global force_curveX, force_curveY, force_curveZ, forceX, forceY, forceZ,\
        force_curveRX, force_curveRY, force_curveRZ, forceRX, forceRY, forceRZ,\
        jPos_1_curve, jPos_2_curve, jPos_3_curve, jPos_4_curve, jPos_5_curve, jPos_6_curve, \
        position_1, position_2, position_3, position_4, position_5, position_6, \
        jV1_curve, jV2_curve, jV3_curve, jV4_curve, jV5_curve, jV6_curve, \
        velocity_1, velocity_2, velocity_3, velocity_4, velocity_5, velocity_6, \
        c, dr, ptr, cur_length

    #get tcp_force value
    force = dr.tcp_force
    position = dr.q_actual
    velocity = dr.qd_actual

    if cur_length >= 500:
        forceX = np.delete(forceX, 0)
        forceY = np.delete(forceY, 0)
        forceZ = np.delete(forceZ, 0)
        forceRX = np.delete(forceRX, 0)
        forceRY = np.delete(forceRY, 0)
        forceRZ = np.delete(forceRZ, 0)
        position_1 = np.delete(position_1, 0)
        position_2 = np.delete(position_2, 0)
        position_3 = np.delete(position_3, 0)
        position_4 = np.delete(position_4, 0)
        position_5 = np.delete(position_5, 0)
        position_6 = np.delete(position_6, 0)
        velocity_1 = np.delete(velocity_1, 0)
        velocity_2 = np.delete(velocity_2, 0)
        velocity_3 = np.delete(velocity_3, 0)
        velocity_4 = np.delete(velocity_4, 0)
        velocity_5 = np.delete(velocity_5, 0)
        velocity_6 = np.delete(velocity_6, 0)

    #append new data to the end of each array
    forceX = np.append(forceX, force[0])
    cur_length = len(forceX)
    forceY = np.append(forceY, force[1])
    forceZ = np.append(forceZ, force[2])
    forceRX = np.append(forceRX, force[3])
    forceRY = np.append(forceRY, force[4])
    forceRZ = np.append(forceRZ, force[5])

    position_1 = np.append(position_1, position[0])
    position_2 = np.append(position_2, position[1])
    position_3 = np.append(position_3, position[2])
    position_4 = np.append(position_4, position[3])
    position_5 = np.append(position_5, position[4])
    position_6 = np.append(position_6, position[5])

    velocity_1 = np.append(velocity_1, velocity[0])
    velocity_2 = np.append(velocity_2, velocity[1])
    velocity_3 = np.append(velocity_3, velocity[2])
    velocity_4 = np.append(velocity_4, velocity[3])
    velocity_5 = np.append(velocity_5, velocity[4])
    velocity_6 = np.append(velocity_6, velocity[5])

    #send data to the plot subwindow
    force_curveX.setData(forceX)
    force_curveY.setData(forceY)
    force_curveZ.setData(forceZ)
    force_curveRX.setData(forceRX)
    force_curveRY.setData(forceRY)
    force_curveRZ.setData(forceRZ)

    jPos_1_curve.setData(position_1)
    jPos_2_curve.setData(position_2)
    jPos_3_curve.setData(position_3)
    jPos_4_curve.setData(position_4)
    jPos_5_curve.setData(position_5)
    jPos_6_curve.setData(position_6)

    jV1_curve.setData(velocity_1)
    jV2_curve.setData(velocity_2)
    jV3_curve.setData(velocity_3)
    jV4_curve.setData(velocity_4)
    jV5_curve.setData(velocity_5)
    jV6_curve.setData(velocity_6)

#    print('Force on X axis: %.2f' % force[0], end='  ')
#    print('Force on Y axis: %.2f' % force[1], end='  ')
#    print('Force on Z axis: %.2f' % force[2])

    if ptr == 0:
        c.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr += 1

#using the QtTimer to do something like a loop here
timer = QtCore.QTimer()
timer.timeout.connect(dr.robot_update)
timer.timeout.connect(plot_update)
timer.start(0)

if __name__ == '__main__':
    import sys
    # create a QtApp property for this program, this is very important
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()