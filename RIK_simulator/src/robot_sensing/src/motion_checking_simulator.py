#!/usr/bin/env python
# license removed for brevity
# code: -*- utf-8 -*-
# author: hwang
# created on: 06/14/2017
# modified on: 07/20/2017
import rospy
import numpy as np
import sys
import csv
import random
import math
import argparse
import json

import tf as tf_rev
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from pyqtgraph.Qt import QtGui, QtCore

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib import colors as mcolors
import matplotlib.pyplot as plt
import pyqtgraph as pg

from transformations.transformations import quaternion_disp, quaternion_log, quaternion_exp, euler_from_quaternion
from predefined_params import *

# motion switches
IS_FILTERED = False
IS_REVISED = True
IS_LAST_MOTION = False

# plotting swithes
TEST_RELAX = False

# global vars
LBD_PLAY_BACK_BASE_DIR = '/home/hwang/log_lbd_playback/csv_file/'
PARAM_RT_DIR_BASE = '/home/hwang/log_lbd_playback/param_log_file/'
PREDEFINED_POS_OBJ_THRESHOLD = 0.0001
topic = 'visualization_marker_array'
topic_2 = 'exact/visualization_marker_array'
topic_3 = 'ipl/visualization_marker_array'

objects = ('s_p', 's_l', 'elb', 'w_1', 'w_2', 'w_3')

# set dict for matplotlib color lists
colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)

# ROS settings
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
# add more marker arrays for other IK method
publisher_2 = rospy.Publisher(topic_2, MarkerArray, queue_size=10)
publisher_3 = rospy.Publisher(topic_3, MarkerArray, queue_size=10)

pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
broadCaster = tf_rev.TransformBroadcaster()
broadCaster_2 = tf_rev.TransformBroadcaster()
broadCaster_3 = tf_rev.TransformBroadcaster()

#pub = rospy.Publisher("joint_states", JointState, queue_size=10)
rospy.init_node("test_playback_node", anonymous=True)

# define markerArray type to show trajecotry
markerArray = MarkerArray()
markerArray_2 = MarkerArray()
markerArray_3 = MarkerArray()

# define jointState type to publish real robot states
js = JointState()
js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
js.position = [ 3.47172217e+00,  -2.14456987e+00,  -1.91532757e+00,   9.18540937e-01, -3.30283928e-01,  -3.21397317e-04]

# Only used for distinguish from file names
if IS_LAST_MOTION:
    JS_FILE_NAME = "js_mg_627_2_rev.csv"
    MOTION_FILE_NAME = "mg_627_2_rev.csv"
    PARAM_FILE_NAME = "mg_627_2_rev.csv"
    ERR_FILE_NAME = "err_mg_627_2_rev.csv"
else:
    if IS_FILTERED:
        JS_FILE_NAME = "js_test_traj_1_fil.csv"
        MOTION_FILE_NAME = "test_traj_1_fil.csv"
        PARAM_FILE_NAME = "test_traj_1_fil.csv"
        ERR_FILE_NAME = "err_test_traj_1_rev.csv"

    elif IS_REVISED:
        
        JS_FILE_NAME = "../../../js_crappy_motion_0_rev_mod36.csv"
        MOTION_FILE_NAME = "crappy_motion_0_rev.csv"
        PARAM_FILE_NAME = "crappy_motion_0_rev_mod36.csv"
        ERR_FILE_NAME = "../../../err_crappy_motion_0_rev_mod36.csv"

        JS_FILE_NAME_EXACT = "../../../js_crappy_motion_0_rev_exact3.csv"
        #MOTION_FILE_NAM = "crappy_motion_0_rev.csv"
        PARAM_FILE_NAME_EXACT = "../../../crappy_motion_0_rev_exact3.csv"
        ERR_FILE_NAME_EXACT = "../../../err_crappy_motion_0_rev_exact3.csv"
        TEST_RELAX = False

        JS_FILE_NAME_IPL = "../../../js_crappy_motion_0_rev_ipl.csv"
        #MOTION_FILE_NAM = "crappy_motion_0_rev.csv"
        PARAM_FILE_NAME_IPL = "crappy_motion_0_rev_ipl.csv"
        ERR_FILE_NAME_IPL = "../../../err_crappy_motion_0_rev_ipl.csv"
        TEST_RELAX = False
        
    else:  
        JS_FILE_NAME = "js_test_traj_1.csv"
        MOTION_FILE_NAME = "test_traj_1.csv"
        PARAM_FILE_NAME = "test_traj_1.csv"
        ERR_FILE_NAME = "err_test_traj_1_rev.csv"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def find_closest(A, target):
    #A must be sorted
    idx = A.searchsorted(target)
    idx = np.clip(idx, 1, len(A)-1)
    left = A[idx-1]
    right = A[idx]
    idx -= target - left < right - target
    return idx

def load_motion_data(global_dir=None, data_type=None):
    '''type indicate data type of csv files. kin stands for pos and orientation
       gripper stands for encoder value of the tongs'''
    csv_times = []
    csv_pos = []
    csv_pos_fake = []
    csv_quat = []
    csv_velocity = []
    counter_ = 0
    with open(LBD_PLAY_BACK_BASE_DIR+global_dir, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            tmp_pos_list = row[1].lstrip('[').rstrip(']').split(',')
            tmp_quat_list = row[2].lstrip('[').rstrip(']').split(',')
            csv_times.append(float(row[0]))
            csv_pos.append([float(item) for item in tmp_pos_list])
#           csv_pos_fake.append([float(item)+0.06 if idx==2 else float(item) for idx, item in enumerate(tmp_pos_list)])
            csv_quat.append([float(item) for item in tmp_quat_list])
            if counter_ == 0:
                csv_velocity.append(0)
            else:
                csv_velocity.append(np.linalg.norm(np.subtract(csv_pos[counter_],csv_pos[counter_-1]))/(csv_times[counter_]-csv_times[counter_-1]))
            counter_+=1
    return np.array(csv_times), csv_pos, csv_quat, csv_pos_fake, csv_velocity

def load_param_data(global_dir=None):
    params_tmp = []
    with open(PARAM_RT_DIR_BASE+global_dir, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            params_tmp.append([float(item) for item in row])
    return params_tmp

def load_joint_states(file_dir="js_test_traj_1_fil.csv"):
    joint_angle_table = []
    with open(file_dir, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            joint_angle_table.append([float(r) for r in row])
    return joint_angle_table

def load_ee_status(file_dir="js_test_traj_1_fil.csv"):
    '''
    load data of end effector for real time check of the error
    '''
    ee_pos = []
    ee_quat = []
    row_counter = 0
    with open(file_dir, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            if row_counter == 0:
                row_counter+=1
                continue
            else:
#                ee_pos.append([float(r)+0.06 if r_idx==0 else float(r) for r_idx, r in enumerate(row[0:3])])
                ee_pos.append([float(r) for r in row[0:3]])
                ee_quat.append([float(r) for r in row[3:]])
            row_counter+=1
    return ee_pos, ee_quat

def get_robot_configuration_change(js_table="err_traj_1_rev.csv"):
    config_change_list = []
    for js_idx, js_state in enumerate(js_table):
        if js_idx == 0:
            config_change_list.append(0)
        else:
            config_change_list.append(np.linalg.norm(np.subtract(js_state, js_table[js_idx-1])))
        if config_change_list[js_idx] >= 0.45:
                print(js_idx)
                print('------------------------------------------------------------------------')
    return config_change_list

def check_fail_frames(params_list=None):
    fail_frames_indices = []
    for param_index, params in enumerate(params_list):
        if params[0] >= PREDEFINED_POS_OBJ_THRESHOLD:
            fail_frames_indices.append(param_index)
    return fail_frames_indices

def get_json_dict(config_change_list, joint_angle_table, predefiend_self_collision_frames):
    assert len(config_change_list) == len(joint_angle_table)
    ret_dict = {}
    config_changes = []
    self_colisions = []
    elbow_flips = []

    for i in range(len(config_change_list)):
        if config_change_list[i] >= 0.45: 
            tmp_config_change = 1
            config_changes.append(i)
        else: 
            tmp_config_change = 0
        if i in predefiend_self_collision_frames: 
            tmp_self_colision = 1
            self_colisions.append(i)
        else: 
            tmp_self_colision = 0
        if joint_angle_table[i][2]<-0.01: 
            tmp_elbow_flip = 1
            elbow_flips.append(i)
        else: 
            tmp_elbow_flip = 0
        ret_dict['frame: {frame_id}'.format(frame_id=i)] = {'elbow flip: ' : tmp_elbow_flip, 
        'robot_configration flipping: ': tmp_config_change, 'self collision: ' : tmp_self_colision}
    return ret_dict, config_changes, self_colisions, elbow_flips

def check_motion_mapping_err(obj_pos=None, obj_quat=None, ee_pos=None, ee_quat=None):
    '''check performance of exact IK solver'''
    pos_err_list = []
    quat_err_list = []
    fixed_quat_for_plot = []
    # check for data length
    assert len(obj_pos) == len(obj_quat)
    assert len(obj_pos) == len(ee_pos) 
    for idx in range(len(obj_pos)):
        tmp_quat = [-elem for elem in ee_quat[idx]]
        metric_0 = np.linalg.norm(np.subtract(obj_quat[idx], ee_quat[idx]))
        metric_1 = np.linalg.norm(np.subtract(obj_quat[idx], tmp_quat))
        if metric_0 > metric_1:
            fixed_quat_for_plot.append(tmp_quat)
        else:
            fixed_quat_for_plot.append(ee_quat[idx])
        pos_err_list.append(np.linalg.norm(np.subtract(obj_pos[idx], ee_pos[idx])))
        # transform obj quart and ee quart to exponential map space
        '''
        v_obj = quaternion_log(obj_quat[idx])
        v_ee = quaternion_log(ee_quat[idx])
        q_exp_obj = quaternion_exp(v_obj)
        q_exp_ee = quaternion_exp(v_ee)'''
        a_obj=euler_from_quaternion(obj_quat[idx])
        a_ee = euler_from_quaternion(ee_quat[idx])
        #quat_err_list.append(np.linalg.norm(quaternion_disp(obj_quat[idx], ee_quat[idx])))
        #quat_err_list.append(np.linalg.norm(quaternion_disp(q_exp_obj, q_exp_ee)))
        a_ee_list=[]
        if np.linalg.norm(np.subtract(a_obj, a_ee))>0.8:
            for i in range(3):
                tmp_a_ee = [-t if t_idx == i else t for t_idx, t in enumerate(a_ee)]
                a_ee_list.append(np.linalg.norm(np.subtract(a_obj, tmp_a_ee)))
            if min(a_ee_list) > 0.8:
                quat_err_list.append(0)
                continue
            quat_err_list.append(min(a_ee_list))
        else:
            quat_err_list.append(np.linalg.norm(np.subtract(a_obj, a_ee)))
    return pos_err_list, quat_err_list, fixed_quat_for_plot

def fill_marker_array(pos, quat, forbiden_frames, markerArray, publisher, color):
    markerArray=markerArray
    publisher=publisher
    for idx in range(len(pos)):
        marker = Marker()
        marker.header.frame_id = "ur5/world"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        if color == 'g':
            marker.scale.x = 0.005
            marker.scale.y = 0.005
            marker.scale.z = 0.005
        else:
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
        if color == 'y':
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.2
        elif color == 'g':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3
        elif color == 'b':
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0            
            marker.color.a = 0.2
        marker.pose.orientation.w = quat[idx][0]
        marker.pose.orientation.x = quat[idx][1]
        marker.pose.orientation.y = quat[idx][2]
        marker.pose.orientation.z = quat[idx][3]
        marker.pose.position.x = pos[idx][0]
        marker.pose.position.y = pos[idx][1] 
        marker.pose.position.z = pos[idx][2]
        marker.id = idx
        markerArray.markers.append(marker)

def get_ee_js_vel(time_stamp=None, ee_pos=None, js_table=None, csv_pos=None):
    hand_vel = []
    ee_vel = []
    js_vel = []
    js_space_vel = []

    # do some data format checks
    assert len(ee_pos) == len(csv_pos)

    for idx in range(len(ee_pos)):
        tmp_js_vel = []
        if idx == 0:
            hand_vel.append(0.0)
            ee_vel.append(0.0)
            js_space_vel.append(0.0)
            js_vel.append([0.0]*6)
        else:
            ee_vel.append(np.linalg.norm(np.subtract(ee_pos[idx], ee_pos[idx-1]))/(time_stamp[idx]-time_stamp[idx-1]))
            hand_vel.append(np.linalg.norm(np.subtract(csv_pos[idx], csv_pos[idx-1]))/(time_stamp[idx]-time_stamp[idx-1]))
            #js_space_vel.append(np.linalg.norm(np.subtract(js_table[idx], js_table[idx-1]))/(time_stamp[idx]-time_stamp[idx-1]))
            js_space_vel.append(np.linalg.norm(np.subtract(js_table[idx], js_table[idx-1])))
            for j_idx, j in enumerate(js_table[idx]):
                tmp_js_vel.append(abs(j-js_table[idx-1][j_idx])/float(time_stamp[idx]-time_stamp[idx-1]))    
            js_vel.append(tmp_js_vel)
    return ee_vel, js_vel, js_space_vel, hand_vel

class Window(QWidget):
    def __init__(self, parent=None, pos=None, quat=None, raw_vel=None, js_table=None, params=None, forbiden_frames=None, 
        robot_config_change_list=None, ee_pos=None, ee_quat=None, pos_err=None, quat_err=None,
        ee_vel=None, js_vel=None, js_space_vel=None, ee_vel_exact=None, js_vel_exact=None, js_space_vel_exact=None,
        pos_err_exact=None, quat_err_exact=None, hand_vel=None, ee_vel_ipl=None, js_vel_ipl=None, 
        js_space_vel_ipl=None, pos_err_ipl=None, quat_err_ipl=None):
        '''
        pos, quat stands for the position and orientation of tongs in a certain demonstration
        js_table: the joint states solved by RIK
        '''
        super(Window, self).__init__(parent)

        # get init of data tables
        self._pos = pos
        self._orientation = quat
        self._js_table = js_table
        self._params = params
        # use for checking performance
        self._forbiden_frames = forbiden_frames
        self._robot_config_change = robot_config_change_list
        self._ee_pos_list = ee_pos
        self._ee_quat_list = ee_quat
        self._pos_err_list = pos_err
        self._quat_err_list = quat_err
        self._pos_err_list_exact = pos_err_exact
        self._quat_err_list_exact = quat_err_exact
        self._pos_err_list_ipl = pos_err_ipl
        self._quat_err_list_ipl = quat_err_ipl
        self._ee_velocity = ee_vel
        self._js_velocity = js_vel
        self._js_space_velocity = js_space_vel
        self._ee_velocity_exact = ee_vel_exact
        self._js_velocity_exact = js_vel_exact
        self._js_space_velocity_exact = js_space_vel_exact
        self._ee_velocity_ipl = ee_vel_ipl
        self._js_velocity_ipl = js_vel_ipl
        self._js_space_velocity_ipl = js_space_vel_ipl
        self._hand_velocity = hand_vel
        self._raw_vel = raw_vel

        layout = QVBoxLayout()

        # Add label to show some texts
        self.l1 = QLabel("LBD Playback Simulator")
        self.l1.setAlignment(Qt.AlignCenter)
        self.l1.setFont(QFont("Arial", 30))
        layout.addWidget(self.l1)

        self.l6 = QLabel("Self Collision: False")
        self.l6.setAlignment(Qt.AlignLeft)
        self.l6.setFont(QFont("Arial", 15))

        # add pyqtgraph widget
        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.p1 = self.win.addPlot(title="Positions")
        self.vLine_p1 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('y', width=2.0, style=QtCore.Qt.DashLine))
        self.p1.addItem(self.vLine_p1, ignoreBounds=True)
        self.p1.addLegend()

        self.p2 = self.win.addPlot(title="Position Error")
        self.vLine_p2 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('y', width=2.0, style=QtCore.Qt.DashLine))
        self.p2.addItem(self.vLine_p2, ignoreBounds=True)
        self.p2.addLegend()

        self.p3 = self.win.addPlot(title="Hand/EE Velocity")
        self.vLine_p3 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('y', width=2.0, style=QtCore.Qt.DashLine))
        self.p3.addItem(self.vLine_p3, ignoreBounds=True)
        self.p3.addLegend()

        self.win.nextRow()
        self.p4 = self.win.addPlot(title="Orientations")
        self.vLine_p4 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('y', width=2.0, style=QtCore.Qt.DashLine))
        self.p4.addItem(self.vLine_p4, ignoreBounds=True)
        self.p4.addLegend()

        self.p5 = self.win.addPlot(title="Orientations Error")
        self.vLine_p5 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('y', width=2.0, style=QtCore.Qt.DashLine))
        self.p5.addItem(self.vLine_p5, ignoreBounds=True)
        self.p5.addLegend()

        self.p6 = self.win.addPlot(title="JS Velocity")
        self.vLine_p6 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('y', width=2.0, style=QtCore.Qt.DashLine))
        self.p6.addItem(self.vLine_p6, ignoreBounds=True)
        self.p6.addLegend()

        self.sl = QSlider(Qt.Horizontal)
        self.sl.setMinimum(0)
        self.sl.setMaximum(len(self._pos)-1)
        self.sl.setValue(500)
        self.sl.setTickPosition(QSlider.TicksBelow)
        self.sl.setTickInterval(1)

        layout.addWidget(self.win)
        layout.addWidget(self.sl)

        layout.addWidget(self.l6)
        self.sl.valueChanged.connect(self.valuechange)
        self.setLayout(layout)
        self.setWindowTitle("Playback Simulator with UI by HWang")
        self.plot(params=self._params,sl_index=100)

    def slider_response(self, sl_index, forbiden_frames=None):
        global markerArray, publisher, pub, js, broadCaster, broadCaster_2
        marker = Marker()
        markerArray.markers[sl_index].scale.x = 0.02
        markerArray.markers[sl_index].scale.y = 0.02
        markerArray.markers[sl_index].scale.z = 0.02
        markerArray.markers[sl_index].color.a = 1.0
        markerArray.markers[sl_index].color.r = 0.0
        markerArray.markers[sl_index].color.g = 0.0
        markerArray.markers[sl_index].color.b = 1.0
        markerArray.markers[sl_index].type = marker.CUBE
        js.position = self._js_table[sl_index]
        js.header.stamp = rospy.Time.now()
        pub.publish(js)
        publisher.publish(markerArray)
        publisher_2.publish(markerArray_2)
        publisher_3.publish(markerArray_3)
        # change last marker to normal
        markerArray.markers[sl_index].scale.x = 0.01
        markerArray.markers[sl_index].scale.y = 0.01
        markerArray.markers[sl_index].scale.z = 0.01

        markerArray.markers[sl_index].color.a = 0.2
        markerArray.markers[sl_index].color.r = 1.0
        markerArray.markers[sl_index].color.g = 1.0
        markerArray.markers[sl_index].color.b = 0.0

        broadCaster.sendTransform((markerArray.markers[sl_index].pose.position.x, markerArray.markers[sl_index].pose.position.y, markerArray.markers[sl_index].pose.position.z),
                     (markerArray.markers[sl_index].pose.orientation.x, markerArray.markers[sl_index].pose.orientation.y, markerArray.markers[sl_index].pose.orientation.z, markerArray.markers[sl_index].pose.orientation.w),
                     rospy.Time.now(),
                     "Marker/base_link",
                     "ur5/world"
                     )
        broadCaster_2.sendTransform((self._ee_pos_list[sl_index][0], self._ee_pos_list[sl_index][1], self._ee_pos_list[sl_index][2]),
                     (self._ee_quat_list[sl_index][1], self._ee_quat_list[sl_index][2], self._ee_quat_list[sl_index][3], self._ee_quat_list[sl_index][0]),
                     rospy.Time.now(),
                     "EE_Viz/base_link",
                     "ur5/world")
        broadCaster_3.sendTransform((markerArray_3.markers[sl_index].pose.position.x, markerArray_3.markers[sl_index].pose.position.y, markerArray_3.markers[sl_index].pose.position.z),
                     (markerArray_3.markers[sl_index].pose.orientation.x, markerArray_3.markers[sl_index].pose.orientation.y, markerArray_3.markers[sl_index].pose.orientation.z, markerArray_3.markers[sl_index].pose.orientation.w),
                     rospy.Time.now(),
                     "IPL_Marker/base_link",
                     "ur5/world"
                     )                              

    def setLabelValue(self, widget=None ,params=None, sl_index=None, type=None, index=None):
        '''
        if isinstance(params[0], list):
            widget.setText(type+str(params[sl_index][index]))
        else:
            widget.setText(type+str(params[sl_index]))
        '''
        widget.setText(type+sl_index)
    
    def valuechange(self):
        sl_index = self.sl.value()
        self.l1.setText("Frame Index: " + str(sl_index))
        self.slider_response(sl_index=sl_index, forbiden_frames=self._forbiden_frames)
#        self.plot(params=self._params, sl_index=sl_index)
        self.slider_change(sl_index=sl_index)
        if sl_index in predefiend_self_collision_frames:
            self.setLabelValue(widget=self.l6, params=self._robot_config_change, sl_index="True", type='Self Collision: ')
        else:
            self.setLabelValue(widget=self.l6, params=self._robot_config_change, sl_index="False", type='Self Collision: ')

    def plot(self, params=None, sl_index=None):
        ''' plot some random stuff '''
        # random data
        data = params[sl_index][0:2]
        js_state = self._js_table[sl_index][:]

        if not TEST_RELAX:
            plot_structure = [i for i in range(131, 134)]
        else:
            plot_structure = [i for i in range(151, 156)]

        # create an axis
        # positions
        self.curve0 = self.p1.plot([pos[0] for pos in self._pos], pen=pg.mkPen('r', style=QtCore.Qt.DashLine), name='x')
        self.curve1 = self.p1.plot([pos[1] for pos in self._pos], pen=pg.mkPen('g', style=QtCore.Qt.DashLine), name='y')
        self.curve2 = self.p1.plot([pos[2] for pos in self._pos], pen=pg.mkPen('b', style=QtCore.Qt.DashLine), name='z')
        self.curve3 = self.p1.plot([pos[0] for pos in self._ee_pos_list], pen='r', name='ee_x')
        self.curve4 = self.p1.plot([pos[1] for pos in self._ee_pos_list], pen='g', name='ee_y')
        self.curve5 = self.p1.plot([pos[2] for pos in self._ee_pos_list], pen='b', name='ee_z')

        # position err
        self.curve6 = self.p2.plot(self._pos_err_list, pen='r', name='dyn relax')
        self.curve6_ = self.p2.plot(self._pos_err_list_exact, pen='g', name='exact')
        self.curve6_p = self.p2.plot(self._pos_err_list_ipl, pen='c', name='ipl')

        # hand/ee velocity
        self.curve7 = self.p3.plot(self._ee_velocity, pen='m', name='dyn relax')
        self.curve7_ = self.p3.plot(self._ee_velocity_exact, pen='r', name='exact')
        self.curve7_p = self.p3.plot(self._raw_vel, pen='c', name='raw')
        self.curve7_p2 = self.p3.plot(self._ee_velocity_ipl, pen='y', name='ipl')

        # orientations
        self.curve8 = self.p4.plot([q[0] for q in self._orientation], pen=pg.mkPen('c', style=QtCore.Qt.DashLine),name='w')
        self.curve9 = self.p4.plot([q[1] for q in self._orientation], pen=pg.mkPen('r', style=QtCore.Qt.DashLine),name='x')
        self.curve10 = self.p4.plot([q[2] for q in self._orientation], pen=pg.mkPen('g', style=QtCore.Qt.DashLine),name='y')
        self.curve11 = self.p4.plot([q[3] for q in self._orientation], pen=pg.mkPen('b', style=QtCore.Qt.DashLine),name='z')

        self.curve12 = self.p4.plot([q[0] for q in self._ee_quat_list], pen='c', name='ee_w')
        self.curve13 = self.p4.plot([q[1] for q in self._ee_quat_list], pen='r', name='ee_x')
        self.curve14 = self.p4.plot([q[2] for q in self._ee_quat_list], pen='g', name='ee_y')
        self.curve15 = self.p4.plot([q[3] for q in self._ee_quat_list], pen='b', name='ee_z')
        # orientation error
        self.curve16 = self.p5.plot(self._quat_err_list, pen='g', name='dyn relax')
        self.curve16_ = self.p5.plot(self._quat_err_list_exact, pen='r', name='exact')
        self.curve16_p = self.p5.plot(self._quat_err_list_ipl, pen='c', name='ipl')

        # JS Velocity
        self.curve17 = self.p6.plot(self._js_space_velocity, pen='g', name='dyn relax')
        self.curve17_ = self.p6.plot(self._js_space_velocity_exact, pen='r', name='exact')
        self.curve17_ = self.p6.plot(self._js_space_velocity_ipl, pen='m', name='ipl')

        self.vLine_p1.setValue(sl_index)
        self.vLine_p2.setValue(sl_index)
        self.vLine_p3.setValue(sl_index)
        self.vLine_p4.setValue(sl_index)
        self.vLine_p5.setValue(sl_index)
        self.vLine_p6.setValue(sl_index)

    def slider_change(self, sl_index=None):
        self.vLine_p1.setValue(sl_index)
        self.vLine_p2.setValue(sl_index)
        self.vLine_p3.setValue(sl_index)
        self.vLine_p4.setValue(sl_index)
        self.vLine_p5.setValue(sl_index)
        self.vLine_p6.setValue(sl_index)

def main(mode=None):
    if JS_FILE_NAME == "../../../js_crappy_motion_0_rev_mod3.csv":
        predefiend_self_collision_frames = range(790,821)+range(890,904)+range(1200,1300)
    else:
        predefiend_self_collision_frames = []

    time_stamp, csv_pos, csv_quat, csv_pos_fake, csv_velocity = load_motion_data(global_dir=MOTION_FILE_NAME)
    ee_pos_array, ee_quat_array = load_ee_status(file_dir=ERR_FILE_NAME)
    ee_pos_array_exact, ee_quat_array_exact = load_ee_status(file_dir=ERR_FILE_NAME_EXACT)
    ee_pos_array_ipl, ee_quat_array_ipl = load_ee_status(file_dir=ERR_FILE_NAME_IPL)

    pos_err, quat_err, fixed_quat=check_motion_mapping_err(obj_pos=csv_pos, obj_quat=csv_quat, ee_pos=ee_pos_array, ee_quat=ee_quat_array)
    pos_err_exact, quat_err_exact, fixed_quat=check_motion_mapping_err(obj_pos=csv_pos, obj_quat=csv_quat, ee_pos=ee_pos_array_exact, ee_quat=ee_quat_array_exact)
    pos_err_ipl, quat_err_ipl, fixed_quat=check_motion_mapping_err(obj_pos=csv_pos, obj_quat=csv_quat, ee_pos=ee_pos_array_ipl, ee_quat=ee_quat_array_ipl)

    params = load_param_data(global_dir=PARAM_FILE_NAME)[1:]
    joint_angle_table = load_joint_states(JS_FILE_NAME)
    joint_angle_table_exact = load_joint_states(JS_FILE_NAME_EXACT)
    joint_angle_table_ipl = load_joint_states(JS_FILE_NAME_IPL)
    # ee vel should be a #frame * 1 vector, js vel should be a #frame * #DOF matrix
    ee_vel, js_vel, js_space_vel, hand_vel = get_ee_js_vel(time_stamp=time_stamp, ee_pos=ee_pos_array, js_table=joint_angle_table, csv_pos=csv_pos)
    ee_vel_exact, js_vel_exact, js_space_vel_exact, _ = get_ee_js_vel(time_stamp=time_stamp, ee_pos=ee_pos_array_exact, js_table=joint_angle_table_exact, csv_pos=csv_pos)
    ee_vel_ipl, js_vel_ipl, js_space_vel_ipl, _ = get_ee_js_vel(time_stamp=time_stamp, ee_pos=ee_pos_array_ipl, js_table=joint_angle_table_ipl, csv_pos=csv_pos)

    config_change_list = get_robot_configuration_change(js_table=joint_angle_table)
    forbiden_frames = check_fail_frames(params_list=params)
    fill_marker_array(csv_pos, csv_quat, forbiden_frames, markerArray, publisher, color='y')
    fill_marker_array(ee_pos_array_exact, ee_quat_array_exact, forbiden_frames, markerArray_2, publisher_2, color='g')
    fill_marker_array(ee_pos_array_ipl, ee_quat_array_ipl, forbiden_frames, markerArray_3, publisher_3, color='b')

    # define two mode here, one is with look into the data, the other is just playback the motion on simulator
    # analysis mode
    if mode == 'a':
        app = QApplication(sys.argv)
        ex = Window(pos=csv_pos, quat=csv_quat, raw_vel=csv_velocity, js_table=joint_angle_table, params=params, 
                    forbiden_frames=forbiden_frames, robot_config_change_list=config_change_list, 
                    ee_pos=ee_pos_array, ee_quat=fixed_quat, pos_err=pos_err, quat_err=quat_err,
                    ee_vel=ee_vel, js_vel=js_vel, js_space_vel=js_space_vel,
                    ee_vel_exact=ee_vel_exact, js_vel_exact=js_vel_exact, js_space_vel_exact=js_space_vel_exact,
                    pos_err_exact=pos_err_exact, quat_err_exact=quat_err_exact, hand_vel=hand_vel,
                    ee_vel_ipl=ee_vel_ipl, js_vel_ipl=js_vel_ipl, js_space_vel_ipl=js_space_vel_ipl,
                    pos_err_ipl=pos_err_ipl, quat_err_ipl=quat_err_ipl)
        ex.show()
        sys.exit(app.exec_())
    # only playback
    elif mode == 'p':
        # load gripper data from csv
        init_state = joint_angle_table[0]
        # move to the initial joint state
        js.position = init_state
        js.header.stamp = rospy.Time.now()
        pub.publish(js)
        rospy.sleep(1)
        # convert time from nsec to sec, start from 0
        times_np_ur = np.array(time_stamp)
    
        tDiv_ = 1000000000.0
        endTime_ur = times_np_ur[-1]
        startTime = float(rospy.Time.now().to_nsec())
        currTime = 0.0
        while not currTime >= endTime_ur and not rospy.is_shutdown():
            currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv_
            idx_ur = find_closest(times_np_ur, currTime)

            js_cmd = joint_angle_table[idx_ur]
            # this is quite important
            rospy.sleep(0.004)
            # do something on the gripper
            
            # move the robot
            js.position = js_cmd
            js.header.stamp = rospy.Time.now()
            pub.publish(js)
            currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv_
    # checking mode:
    elif mode == 'c':
        json_dict, config_changes, self_colisions, elbow_flips = get_json_dict(config_change_list, joint_angle_table, predefiend_self_collision_frames)

        with open('../../../json/'+MOTION_FILE_NAME.rstrip('.csv')+'.json', 'w') as fp:
            json.dump(json_dict, fp)
        print("IK results checking is Done: ")
        if len(config_changes) ==0 and len(self_colisions) ==0 and len(elbow_flips)==0:
            status = 'acceptable'
        else:
            status = 'unacceptable'
        if status == 'unacceptable':
            print(bcolors.WARNING + "The checking result for this motion is: {}".format(status)+ bcolors.ENDC)
        else:
            print(bcolors.OKGREEN + "The checking result for this motion is: {}".format(status)+ bcolors.ENDC)

        if status == "acceptable":
            print("Please move on to play it back on real robot!")
        else:
            print 
            print("Large robot config change frames: ")
            print(config_changes)
            print
            print("Elbow flip frames: ")
            print(elbow_flips)
            print
            print("Self collision frames: ")
            print(self_colisions)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # this can be c for checking; for p for just play back
    parser.add_argument('-m', '--mode', dest='checking_mode', help='control the mode of checking tool', required=True)
    given_args = parser.parse_args()
    mode = given_args.checking_mode
    main(mode=mode)