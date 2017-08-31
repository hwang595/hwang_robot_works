# lbd_comparisons
This repo contains a pending project that we wanted to make comparison among kinesthetic teaching vs direct teaching vs teleoperation control of the robot.

# kinesthetic teaching:
For this method, everything is done in this repo. The data collection code is in `free_drive_demo`, which let you press the wireless button to enter free drive mode of ur5 robot, and with control access to the gripper.
For playback of kinesthetic teaching, the folder `src/play_back` contains everything you need. Please make sure you save the data you collect in appropriate directory by reading the code before you wish to play it back on the real robot.

# direct teaching:
It's an almost done project, I've already done most part of the work, what remaining is just test on human input motions as many as possible.
Here are descriptions about the works I've done:
1. figured out how to solve `exact inverse kinematics`. "exact IK" meas, for each frame, we don't care if the solution returned from IK solver is smooth. What we care is if position and orientation match perfectly to objective motion (human motion). This exact solution should always be able to find unless the robot is out of it's reachable range.
What I did to find exact IK solution is using `random start` strategy: the IK solver firstly tried the initial state it used for the entire motion, if that is still not working (i.e. the position and orientation error is still high), then it randomly generated a initial state near that original initial state. I did that since I want a reasonable starting point for the optimization. Then the IK is running until it found a good solution, or if it can't find any good solution in n iterations (say 80), then it returns the "closest" solution.
The performance of random start is like:
![alt text](https://github.com/hwang595/hwang_robot_works/blob/master/RIK_simulator/pictures/random_start_for_exact_ik.png)

2. figured out dynamic RIK based on the exact IK solutions: if the exact IK told me that at some frames I it can't reach a "perfect" solution. Then, some relaxation (reduce weight on orientation terms) need to be done on orientation term in the IK optimization problem. In the current version, I totally turn off the orientation term (you may need to fix this later), and smooth the orientation weight signal among the whole motion frames.
