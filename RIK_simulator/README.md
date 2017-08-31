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

2. figured out dynamic RIK based on the exact IK solutions: if the exact IK told me that at some frames I it can't reach a "perfect" solution. Then, some relaxation (reduce weight on orientation terms) need to be done on orientation term in the IK optimization problem. In the current version, I totally turn off the orientation term (you may need to fix this later), and smooth the orientation weight signal among the whole motion frames. The weight of orientation will be something like:

![alt text](https://github.com/hwang595/hwang_robot_works/blob/master/RIK_simulator/pictures/RIK_dynamic_relax.png)

3. This work is somehow an extension of Daniel Rakita's RIK work. I added some extra strategy in his framework, like avoid elbow flip issue of the robot, and avoid self collision issue of the robot. All these solution is embedded in `src/lbd_playback/bin/Relaxed_IK_Solver/RelaxedIK/constraint.py`, in the class `ElbowFlipAvoidenceConstraint` and `SelfCollisionAvoidenceConstraint`.

4. to make us see what's going on in a "failed" motion, I built motion simulator based on `RVIZ`, and motion checking tool(an UI wrote in PyQt and PyQtGraph). 
To run them, you need firstly `roslaunch` the whole project by:
```
roslaunch startup ur5_setup_test.launch
```
please note that, for now the `ur5_setup.launch` include the `robotiq` gripper and force torque sensor in it, but they will somehow conflict with the motion checking tool. If you need to follow my work and figure this out, I can work with you on this.
Next, running thie following command to run the motion checking tool:
```
rosrun robot_sensing motion_checking_simulator.py -m [mode]
```
This may not work directly, because you need to collect motion data and run the IK solver first.

There are three mode in this motion chekcing tool:
a (Analysis mode): will open the UI and make comparsions for you among different RIK methods.
c (Checking mode): will run the test and tell you if your solution is good to playback on robot or not. in currenly version it will only check for `elbow flip`, `self collision` and `robot configuration jumpping` (if the robot motion is smooth).
