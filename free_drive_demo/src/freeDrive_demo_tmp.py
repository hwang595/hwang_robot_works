#code: utf-8 -*-
#author: hwang
#created on: 05/23/2017
#modified on: 05/31/2017
'''this is just a temp solution for Fri demo'''

import evdev
import pickle
import serial
import time
import socket
import subprocess
import signal
from threading import Thread
"""
This module helps communicate with the UR robot via xmlrpc. The class is designed to contain all the remote procedure
calls (prefixed by rpc) and helper functions for the server side. The server runs here and the client runs on the UR
robot. The UR continuously updates it's controller parameters through these rpc calls. The rpc server runs in a
seperate thread. Once the main thread is complete the server thread exits.

"""
GRIPPER_STATUS_BASE = "/home/hwang/log_files/gripper_status/"
FREEDRIVE_STATUS_BASE = "/home/hwang/log_files/freeDrive_status/"
ROS_BAG_DIR = "/home/hwang/log_files/kinematics_demo/bag_files/"
A = '/home/hwang/log_files/kinematics_demo/bag_files/bag_file_test2.bag'
#ROS_BAG_DIR = '/home/hwang/log_files/kinematics_demo/bag_files/bag_file_hahahahaha.bag'
class keyEventHandler:
    """
    rpc server through which we communicate with the UR robot, clinet running on the robot keep
    hearing state from this class
    """
    def __init__(self, socket):
    	"""
    	_event_up/down/left/right correspond to button press event
    	when button pressed down, _event_motion were going to be sett True
    	when the button release, _event_motion were going to be sett False
    	"""
        # define up(+), down(-), left(<), right(>), mid(|>)
	    # to be five button on the wireless button device

        self._event_up = True
        self._event_down = True
        self._event_left = False
        self._event_right = False
        self._event_mid = True
	# False for odd, True for even, this member only used for double clik the button
	# say first time to close gripper, second time for open the gripper
	self._oddEven_status_up = False
	self._oddEven_status_mid = False
	self._oddEven_status_down = False
	self._record_start = False

        #key code for five keys: up-115, down-114, left-165, right-163, mid-164
        self._status_dict = {115:self._event_up,
                             114:self._event_down,
                             165:self._event_left,
                             163:self._event_right,
                             164:self._event_mid}
	self._socket = socket

    def get_key_event(self):
        with self.rlock:
            if self._status_dict[115]:
                key_event = 0
            elif self._status_dict[114]:
                key_event = 1
            elif self._status_dict[165]:
                key_event = 2
            elif self._status_dict[163]:
                key_event = 3
            elif self._status_dict[164]:
                key_event = 4
            # if neither keys are pressed, something should happen
            # , which is probably wait
            else:
                key_event = 88
        return key_event

    def detect_button_event(self, device=evdev.InputDevice('/dev/input/event4')):
        '''Let + on the bluetooth button control entering free drive mode'''
        for event in device.read_loop():
            # to handle key event for up key on the button
            if event.type == evdev.ecodes.EV_KEY:
                self.handle_key_event(event_code=event.code, event_value=event.value)

    def handle_key_event(self, event_code, event_value):
        #event.code for + is 115, event.val: 01 for press and 00 for release
        key_event_status = self._status_dict[event_code]
        if event_value == 1 and not key_event_status:
            # button is released, and detect press event
            self._status_dict[event_code] = True
            print "key pressed!"
        elif event_value == 0 and key_event_status:
            # button was pressed
            self._status_dict[event_code] = False
	    # one press-release completed
            print "key released!"
	    if event_code == 115:
		if not self._oddEven_status_up:
		    dump_to_pkl(file_dir=GRIPPER_STATUS_BASE+'gripper_status.pkl', var='close')
		    self._oddEven_status_up = not self._oddEven_status_up
		else:
		    dump_to_pkl(file_dir=GRIPPER_STATUS_BASE+'gripper_status.pkl', var='open')
		    self._oddEven_status_up = not self._oddEven_status_up
	    elif event_code == 164:
		if not self._oddEven_status_mid:
		    dump_to_pkl(file_dir=FREEDRIVE_STATUS_BASE+'freeDrive_status.pkl', var='odd_middle_click')
		    self._oddEven_status_mid = not self._oddEven_status_mid
		    print("Enter Free Drive Mode!")
		    self._socket.send("set_digital_out(1, True)" + "\n")
		    #record_rosbag_files()
		    # need to block to open another thread here:
		    if not self._record_start:
			# flip the flag
			self._record_start = not self._record_start
#			subprocess.Popen('rosbag record -a -O test_aaa.bag', shell=True, stdin=subprocess.PIPE, cwd=ROS_BAG_DIR)
		    	start_recording_in_parallel()
		else:
		    dump_to_pkl(file_dir=FREEDRIVE_STATUS_BASE+'freeDrive_status.pkl', var='even_middle_click')
		    self._oddEven_status_mid = not self._oddEven_status_mid
		    print("End Free Drive Mode!")
		    self._socket.send("set_digital_out(1, False)" + "\n")
		    #subprocess.send_signal(signal.SIGINT)
	    elif event_code == 114:
		if not self._oddEven_status_up:
		    dump_to_pkl(file_dir=GRIPPER_STATUS_BASE+'logger_status.pkl', var='start')
		    self._oddEven_status_down = not self._oddEven_status_down
		else:
		    dump_to_pkl(file_dir=GRIPPER_STATUS_BASE+'logger_status.pkl', var='stop')
		    self._oddEven_status_down = not self._oddEven_status_down

def dump_to_pkl(file_dir=None, var=None):
    output = open(file_dir, 'wb')
    pickle.dump(var, output, -1)

def start_recording_in_parallel():
    thread = Thread(target=rosbag_record)
    thread.start()
    thread.join()

def rosbag_record():
    print "start recording bag files"
    subprocess.call(['python', 'test_sub_process.py'])    

if __name__ == "__main__":
    print "start to hear the key event: "
    print "hearing the key event ..."
    HOST = "192.168.1.108"    
    PORT = 30002  # The same port as used by the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    device = evdev.InputDevice('/dev/input/event4')
    keh = keyEventHandler(socket=s)
    keh.detect_button_event(device)
