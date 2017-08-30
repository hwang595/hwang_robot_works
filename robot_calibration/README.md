# calibration between robot and mocap
This work mainly implement calibration between ur5 robot and motion capture. Two sequence of position data (two trajectory) wil be needed to feed in this code. And a transformation matrix will be returned.

The core function used in this code is `transformations.superimposition_matrix`, which make it possible to get transformation matrix without orientation information.

Running the code by `python calibration_new.py -f [ros_bag_file_dir]`, then the code will plot calibration result so you can check it:
![alt text](https://github.com/hwang595/hwang_robot_works/blob/master/robot_calibration/image/calibration_result.png)
also, the code will print the transformation matrix like:
```
[[-0.99804964  0.06126768 -0.01196614 -0.23600125]
 [-0.06113039 -0.9980633  -0.01152046 -0.51506781]
 [-0.0126488  -0.01076649  0.99986204 -0.15295339]
 [ 0.          0.          0.          1.        ]]
```
