from transformations import quaternion_matrix, quaternion_from_matrix

import numpy as np

# in form of x, y, z
predefined_pos = [-0.363513708115, 0.325969904661, -0.00278891064227]
# in form of w, x, y, z
predefined_quat = [0.73178178072, -0.681429624557, 0.00782157666981, 0.00937978643924]

# what we want is M * robot_m + t = mocap_m
mocap_m = np.array(quaternion_matrix(predefined_quat))[0:3]
mocap_m = mocap_m[:, 0:3]

M = np.zeros((3, 3))
trans_r_moc = np.zeros((4, 4))

'''
for i in range(3):
	M[i,:] = mocap_m[i,:] - np.array(predefined_pos)
'''

for i in range(3):
	M[i,:] = mocap_m[i,:]

fake_robot_motion = np.eye(4)

for i in range(4):
	if i <= 2:
		trans_r_moc[i, 0:3] = M[i, :]
		trans_r_moc[i, 3] = predefined_pos[i]
	else:
		trans_r_moc[i, 0:3] = np.array([0, 0, 0])
		trans_r_moc[i, 3] = 1

#print trans_r_moc
transformed_r = np.dot(trans_r_moc, fake_robot_motion)

print transformed_r
print
print quaternion_from_matrix(transformed_r[0:3, 0:3])
print
print np.linalg.inv(transformed_r)
print 
print quaternion_from_matrix(np.linalg.inv(transformed_r[0:3, 0:3]))