# calibration between robot and mocap
This work mainly implement calibration between ur5 robot and motion capture. Two sequence of position data (two trajectory) wil be needed to feed in this code. And a transformation matrix will be returned.

The core function used in this code is `transformations.superimposition_matrix`, which make it possible to get transformation matrix without orientation information.
