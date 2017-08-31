# lbd_comparisons
This repo contains a pending project that we wanted to make comparison among kinesthetic teaching vs direct teaching vs teleoperation control of the robot.

# kinesthetic teaching:
For this method, everything is done in this repo. The data collection code is in `free_drive_demo`, which let you press the wireless button to enter free drive mode of ur5 robot, and with control access to the gripper.
For playback of kinesthetic teaching, the folder `src/play_back` contains everything you need. Please make sure you save the data you collect in appropriate directory by reading the code before you wish to play it back on the real robot.

# direct teaching:
It's an almost done project, I've already done most part of the work, what remaining is just test on human input motions as many as possible.
Here are descriptions about the works I've done:
1. figured out how to solve `exact inverse kinematics`. "exact IK" meas, for each frame, we don't care if the solution returned from IK solver is smooth. What we care is if position and orientation match perfectly to objective motion (human motion). This exact solution should always be able to find unless the robot is out of it's reachable range.
