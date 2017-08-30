# staggered-poses
This work is an extension of staggered poses method in computer animation, the original work can be found at (http://dl.acm.org/citation.cfm?id=1632612).

I also presented this work on poster session of TTI's robotics workshop, the poster can be found at:
(https://drive.google.com/file/d/0BwP87_GG-83QdzdPU1hOOU40OTg/view?usp=sharing)

This code mainly focuses on how to find staggered poses frames (like essential frames) in a human motion with input of differnet channels of signals (force, encoder value, position, and orientation for this current version). After that, the code regenerate a new (cleaner) human motion using the staggered poses frames (by interpolation among them).

Run this code by:
`python src/staggered_poses_moCap_version.py -f [ros_bag_file_dir]`

The code will parse the rosbag file (human motion data) and force synchronization among different channel of signals. The raw and regenerated motion data will be written into `../out_csv_file/raw.csv` and `../out_csv_file/stagPos.csv` respectively.
