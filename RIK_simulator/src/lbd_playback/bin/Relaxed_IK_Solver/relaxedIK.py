__author__ = 'drakita'
# modified by hwang
import sys
import os
import csv
import pickle

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dir_path)

from RelaxedIK.urdf_load import *
from RelaxedIK.objective import *
from RelaxedIK.constraint import *
from RelaxedIK.vars import *
from RelaxedIK.colors import *
import RelaxedIK.transformations as T
import scipy.optimize as O
from RelaxedIK.objective import *
from RelaxedIK.weight_function import *
from RelaxedIK.relaxation_penalty import RelaxationPenalty

#predefined_js_limits = ((-1, 1.7), (-2.1, -1.7), (-2.3, -0.5), (-3.05, -0.06), (None, None), (None, None))
predefined_js_limits = ((-6.20, 6.20), (-4.48, 1.34), (-2.86, 2.86), (-6.20, 6.20), (-6.20, 6.20), (-6.20, 6.20))
backup_init_state = [0.00, -2.7, -1, 0.00, 1.50, 0.00]
ik_solving_iterations = 10

PARAM_RT_DIR_BASE = '/home/hwang/log_lbd_playback/param_log_file/'

def rotate_quat(q, rot, axis):
    rot_mat = T.quaternion_matrix(q)
    if axis == 'x':
        a = rot_mat[0:3,0]
    elif axis == 'y':
        a = rot_mat[0:3,1]
    elif axis == 'z':
        a = rot_mat[0:3,2]
    else:
        raise ValueError('Invalid axis in rotate_quat!')

    rot_w = M.cos(rot/2.0)
    rot_axis = M.sin(rot/2.0)*a
    rot_quat = [rot_w, rot_axis[0], rot_axis[1], rot_axis[2]]
    return T.quaternion_multiply(rot_quat, q)

def get_random_init_state():
    rand_init_state=[]
    for js_val in backup_init_state:
        rand_init_state.append(np.random.normal(js_val, 0.5))
    return rand_init_state

def get_pos_penalty(vars=None, cur_pos_err=0.0, cur_frame_idx=None):
    # do some check first
    if cur_pos_err > vars.get_max_pos_err:
        raise ValueError("current error for frame " + str(cur_frame_idx) + " exceeds max pos err limit")
    else:
        return (vars.get_max_pos_err-cur_pos_err)/vars.get_max_pos_err

def get_orientation_penalty(xopt, vars):
    # calculate ee velocity first
    velMax = vars.handVelMax
    handVel = np.linalg.norm(vars.goal_pos - vars.prev_goal_pos)
    hand_vel_penalty = (max(0.0, velMax - handVel) / velMax) ** 2
    
    js_distance = xopt - np.array(vars.prev_state)
    js_vel=np.linalg.norm(js_distance)**2
    # do some checks first
    if js_vel > vars.get_max_js_vel:
        #raise ValueError("current joint velocity exceeds the js vel limit!")
        merged_penalty = 1.0
        print("Very high penalty here!")
        return merged_penalty
    else:
        js_vel_penalty = (max(0.0, vars.get_max_js_vel - js_vel) / vars.get_max_js_vel) ** 2
    merged_penalty = (0.1 * hand_vel_penalty + 0.9 * js_vel_penalty)

    return merged_penalty

def get_ee_js_vel(xopt, vars, eePos=None):
    # calculate ee velocity first
    #velMax = vars.handVelMax
    ee_vel = np.linalg.norm(np.array(eePos) - np.array(vars.prev_ee_pos))

    js_distance = xopt - np.array(vars.prev_state)
    js_vel=np.linalg.norm(js_distance)**2

    return ee_vel, js_vel

def get_smooth_weight():
    smooth_weights = []
    with open("weights/smooth_weights.csv", 'rb') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            smooth_weights.append(float(row[0]))
    return smooth_weights


class RelaxedIK:
    def __init__(self, (urdf_path, start_joint, end_joint, fixed_ee_joint),
                 full_joint_list=(),
                 init_state=6*[0],
                 rotation_mode = 'displacements', # could be 'absolute' or 'displacements'
                 position_mode = 'displacements', # could be 'absolute' or 'displacements'
                 objectives=(Position_Obj(), Orientation_Obj(), Min_EE_Vel_Obj(), Min_Jt_Vel_Obj()),
                 weight_funcs=(Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight()),
                 weight_priors=(12000.0, 6000.0, 0.0, 0.0),
                 #weight_priors=(12.0, 6.0, 1.0, 3.0),
                 constraints=(),
                 bounds=(),
                 num_frames=0):

        # check inputs ####################################################################################################################
        if (start_joint == '' or end_joint == '') and full_joint_list == ():
            print bcolors.FAIL + 'Invalid robot info.  Must either specify start and end joints or specify full joint list.' + bcolors.ENDC
            raise ValueError('Invalid robot info.')
        if not (rotation_mode == 'displacements' or rotation_mode == 'absolute'):
            print bcolors.FAIL + 'Invalid rotation_mode.  Must be <displacements> or <absolute>.  Exiting.' + bcolors.ENDC
            raise ValueError('Invalid rotation_mode.')
        if not (position_mode == 'displacements' or position_mode == 'absolute'):
            print bcolors.FAIL + 'Invalid position_mode.  Must be <displacements> or <absolute>.  Exiting.' + bcolors.ENDC
            raise ValueError('Invalid position_mode.')
        num_objs = len(objectives)
        if not (num_objs == len(weight_funcs) == len(weight_priors)):
            print bcolors.FAIL + 'Invalid Inputs.  The number of objectives ({}) must be the same as the number' \
                                 'of weight functions ({}) and weight priors ({}).  Exiting.'.format(str(num_objs),
                                                                                                    str(len(weight_funcs)),
                                                                                                    str(len(weight_priors))) + bcolors.ENDC
            raise ValueError('Invalid function arguments.')
        ###################################################################################################################################

        urdf_robot, arm, tree = urdf_load(urdf_path, start_joint, end_joint, full_joint_list, fixed_ee_joint)

        self.urdf_robot = urdf_robot
        self.arm = arm
        self.tree = tree
        self.init_state = init_state
        self.rotation_mode = rotation_mode
        self.position_mode = position_mode
        self.constraints = constraints
        self.numDOF = len(arm.axes)
        if not self.numDOF == len(init_state):
            self.init_state = self.numDOF*[0]
            self.bounds = [tuple((-1000.0,1000.0)) for i in range(0,self.numDOF)]
            print bcolors.WARNING + 'WARNING: Length of init_state does not match number of robot DOFs.  Automatically ' \
                                    'initializing init_state as {}.  This may cause errors.'.format(str(self.init_state)) + bcolors.ENDC
        self.bounds = bounds
        pre_calculated_weights = get_smooth_weight()
        self.vars = Vars(self.arm, self.init_state,objectives,weight_funcs,weight_priors,self.constraints, self.bounds, num_frames=num_frames, pre_calculated_weights=pre_calculated_weights)
        self.constraint_dict = self.__construct_constraint_dict(constraints)
        self.penalty = RelaxationPenalty(self.vars)
        self.penalty_val_merged = [0.0] * num_frames

    def exact_ik_solve(self, goal_pos, goal_quat, prev_state=None, vel_objectives_on=True, verbose_output=False, frame_idx=0):
        '''
        implement of this exact IK to guide us where and how to modify
        :param goal_pos:
        :param goal_quat:
        :param prev_state:
        :return:
        '''

        if self.rotation_mode == 'displacements':
            self.vars.goal_quat = T.quaternion_multiply(goal_quat, self.vars.init_ee_quat)
        elif self.rotation_mode == 'absolute':
            self.vars.goal_quat = goal_quat

        # update current frame id adaptively
        self.vars.current_frame = frame_idx
        self.vars.weight_priors = (12000.0, 6000.0, 0.0, 0.0)
        self.vars.weight_funcs = (Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight())
        #opt_init_state here, empty for no random start
        opt_init_state = []

        init_state_in_progress = []
        solution_in_progress = []
        xopt_in_progress = []

        # flip goal quat if necessary
        disp = np.linalg.norm(T.quaternion_disp(self.vars.prev_goal_quat,self.vars.goal_quat))
        q = self.vars.goal_quat
        if disp > M.pi / 2.0:
            self.vars.goal_quat = [-q[0],-q[1],-q[2],-q[3]]

        if self.position_mode == 'displacements':
            self.vars.goal_pos = np.array(goal_pos) + self.vars.init_ee_pos
        elif self.position_mode == 'absolute':
            self.vars.goal_pos = np.array(goal_pos)

        if prev_state == None:
            initSol = self.vars.prev_state
        else:
            initSol = prev_state

        self.vars.vel_objectives_on = vel_objectives_on

        # just for testing hwang's direct teaching project
        self.bounds = predefined_js_limits

        # Solve #############################################################################################################
        xopt_full = O.minimize(objective_master,initSol,constraints=self.constraint_dict,bounds=self.bounds,args=(self.vars,),method='SLSQP',options={'maxiter':75,'disp':verbose_output})
        #####################################################################################################################
        xopt = xopt_full.x

        if verbose_output:
            print bcolors.OKBLUE + xopt_full + bcolors.ENDC + '\n'

        # check rotation convergence
        frames = self.vars.arm.getFrames(xopt)[1]
        eePos = self.vars.arm.getFrames(xopt)[0][-1]
        eeMat = frames[-1]
        goal_quat = self.vars.goal_quat
        ee_quat = T.quaternion_from_matrix(eeMat)

        #=====================================================================================================
        # check the performance of IK here (how far the solved pos and goal pos)
        pos_distance = np.linalg.norm(np.subtract(eePos, self.vars.goal_pos))
        # this means at this frame the IK solver failed at first time, then we keep changing initial state
        # and solve the IK again
        if pos_distance >= 1e-4:
            for i in range(40):
                if i == 0:
                    initSol = backup_init_state
                else:
                    initSol = get_random_init_state()
                # Solve the IK again for closer solution
                xopt_full_2 = O.minimize(objective_master,initSol,constraints=self.constraint_dict,bounds=self.bounds,args=(self.vars,),method='SLSQP',options={'maxiter':75,'disp':verbose_output})
                xopt_2 = xopt_full_2.x
                # check performance for this iteration
                frames = self.vars.arm.getFrames(xopt_2)[1]
                eePos = self.vars.arm.getFrames(xopt_2)[0][-1]
                pos_distance = np.linalg.norm(np.subtract(eePos, self.vars.goal_pos))
                solution_in_progress.append(pos_distance)
                init_state_in_progress.append(initSol)
                xopt_in_progress.append(xopt_2)
                if pos_distance < 1e-4:
                    break
            pos_distance = min(solution_in_progress)
            xopt = xopt_in_progress[solution_in_progress.index(pos_distance)]
            opt_init_state = init_state_in_progress[solution_in_progress.index(pos_distance)]
            solution_in_progress=[]
            xopt_in_progress = []
            init_state_in_progress = []
            # do not have any solution even run for 40 iterations
            if i == 39:  
                self.penalty.pos_relaxation[self.vars.current_frame]=(get_pos_penalty(vars=self.vars, cur_pos_err=pos_distance, cur_frame_idx=self.vars.current_frame))        
                print(self.vars.current_frame)
                print('=======================================================================================')
                self.vars.out_of_reach_frames.append(self.vars.current_frame)
        self.penalty.orientation_relaxation[self.vars.current_frame]=(get_orientation_penalty(xopt=xopt, vars=self.vars))

        eePos = self.vars.arm.getFrames(xopt)[0][-1]
        eeMat = frames[-1]
        ee_quat = T.quaternion_from_matrix(eeMat)

        ee_quat_tmp=rotate_quat(ee_quat, 1.57075, 'z')
        ee_quat_tmp=rotate_quat(ee_quat_tmp, -1.57075, 'x')

        q = goal_quat
        goal_quat2 = [-q[0],-q[1],-q[2],-q[3]]
        disp = np.linalg.norm(T.quaternion_disp(goal_quat,ee_quat))
        disp2 = np.linalg.norm(T.quaternion_disp(goal_quat2,ee_quat))

        self.vars.prev_state = self.vars.xopt
        self.vars.xopt = xopt
        ee_vel, js_vel=get_ee_js_vel(xopt, self.vars, eePos=eePos)

        #print(self.vars.current_frame, ee_vel/0.01, js_vel)
        #print('=============================================================================================')

        self.vars.prev_ee_pos = self.vars.arm.getFrames(xopt)[0][-1]
        self.vars.prev_ee_quat = T.quaternion_from_matrix(self.arm.getFrames(xopt)[1][-1])
        self.vars.prev_goal_pos = self.vars.goal_pos

        if disp2 < disp:
            pass
            # self.vars.prev_goal_quat = goal_quat2
        else:
            pass
            # self.vars.prev_goal_quat = goal_quat

        self.vars.all_states.append(xopt)
        self.vars.all_ee_pos.append(self.vars.prev_ee_pos)
        self.vars.all_hand_pos.append(self.vars.goal_pos)
        return      

    def solve(self, goal_pos, goal_quat, prev_state=None, vel_objectives_on=True, verbose_output=False, frame_idx=0):
        '''
        :param goal_pos:
        :param goal_quat:
        :param prev_state:
        :return:
        '''

        if self.rotation_mode == 'displacements':
            self.vars.goal_quat = T.quaternion_multiply(goal_quat, self.vars.init_ee_quat)
        elif self.rotation_mode == 'absolute':
            self.vars.goal_quat = goal_quat

        # update current frame id adaptively
        self.vars.current_frame = frame_idx
        if self.vars.current_frame == 0:
            self.prev_state = self.init_state

        if self.vars.current_frame not in self.vars.out_of_reach_frames:
            #self.vars.weight_priors = (12.0, 6.0, 0.1, 0.05)
            self.vars.weight_priors = (12.0, 6.0, 0.1, 1)
        else:
            self.vars.weight_priors = (12.0, 0.0, 0.1, 1)
        self.vars.weight_funcs = (Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight())
        #self.vars.weight_funcs = (Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight())
        #opt_init_state here, empty for no random start
        opt_init_state = []

        init_state_in_progress = []
        solution_in_progress = []
        xopt_in_progress = []

        # flip goal quat if necessary
        disp = np.linalg.norm(T.quaternion_disp(self.vars.prev_goal_quat,self.vars.goal_quat))
        q = self.vars.goal_quat
        if disp > M.pi / 2.0:
            self.vars.goal_quat = [-q[0],-q[1],-q[2],-q[3]]

        if self.position_mode == 'displacements':
            self.vars.goal_pos = np.array(goal_pos) + self.vars.init_ee_pos
        elif self.position_mode == 'absolute':
            self.vars.goal_pos = np.array(goal_pos)


        if prev_state == None:
            initSol = self.vars.prev_state
        else:
            initSol = prev_state

        self.vars.vel_objectives_on = vel_objectives_on

        # just for testing hwang's direct teaching project
        self.bounds = predefined_js_limits

        # Solve #############################################################################################################
#        xopt_full = O.minimize(objective_master,initSol,constraints=self.constraint_dict,bounds=self.bounds,args=(self.vars,),method='SLSQP',options={'maxiter':75,'disp':verbose_output})
#        if self.vars.current_frame == 0:
        xopt_full = O.minimize(objective_master_rev,initSol,constraints=self.constraint_dict,bounds=self.bounds,args=(self.vars,),method='SLSQP',options={'maxiter':75,'disp':verbose_output})
#        else:
#            xopt_full = O.minimize(objective_master_rev,initSol,constraints=self.constraint_dict,bounds=self.bounds,args=(self.vars,),method='SLSQP',options={'maxiter':75,'disp':verbose_output})
        #####################################################################################################################
        xopt = xopt_full.x

        if verbose_output:
            print bcolors.OKBLUE + xopt_full + bcolors.ENDC + '\n'

        # check rotation convergence
        frames = self.vars.arm.getFrames(xopt)[1]
        eePos = self.vars.arm.getFrames(xopt)[0][-1]
        eeMat = frames[-1]
        goal_quat = self.vars.goal_quat
        ee_quat = T.quaternion_from_matrix(eeMat)

        ee_quat_tmp=rotate_quat(ee_quat, 1.57075, 'z')
        ee_quat_tmp=rotate_quat(ee_quat_tmp, -1.57075, 'x')

        with open('err_crappy_motion_0_rev_mod36.csv', 'ab') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(np.concatenate((eePos, ee_quat_tmp), axis=0))

        q = goal_quat
        goal_quat2 = [-q[0],-q[1],-q[2],-q[3]]
        disp = np.linalg.norm(T.quaternion_disp(goal_quat,ee_quat))
        disp2 = np.linalg.norm(T.quaternion_disp(goal_quat2,ee_quat))

        # log info into vars
        self.vars.prev_state = self.vars.xopt
        self.vars.xopt = xopt
        self.log_parameters()

        self.vars.prev_ee_pos = self.vars.arm.getFrames(xopt)[0][-1]
        self.vars.prev_ee_quat = T.quaternion_from_matrix(self.arm.getFrames(xopt)[1][-1])
        self.vars.prev_goal_pos = self.vars.goal_pos

        if disp2 < disp:
            pass
            # self.vars.prev_goal_quat = goal_quat2
        else:
            pass
            # self.vars.prev_goal_quat = goal_quat

        self.vars.all_states.append(xopt)
        self.vars.all_ee_pos.append(self.vars.prev_ee_pos)
        self.vars.all_hand_pos.append(self.vars.goal_pos)
        return xopt

    def log_parameters(self):
        #vars.frames = vars.arm.getFrames(x)
        vars = self.vars
        objectives = vars.objectives
        weight_funcs = vars.weight_funcs
        weight_priors = vars.weight_priors
        objective_sum = 0.0
        weight_tmp = []
        params_log_tmp = []
        for i,o in enumerate(objectives):
            weight_func = weight_funcs[i]
            term_weight = weight_priors[i]*weight_func(vars)
            weight_tmp.append(term_weight)
            objective_sum += term_weight*o(x,vars)
            if i <= 2:
                params_log_tmp.append(o(x,vars))
                if i == 1:
                    params_log_tmp.append(term_weight)
            else:
                params_log_tmp.append(o(x,vars))
        params_log_tmp.append(objective_sum)
        if self.penalty.pos_relaxation[self.vars.current_frame] == 0.0:
            params_log_tmp.append(self.penalty.pos_relaxation[self.vars.current_frame])
        else:
            params_log_tmp.append(1.0-self.penalty.pos_relaxation[self.vars.current_frame])
        if self.penalty.orientation_relaxation[self.vars.current_frame] == 0.0 or\
            self.penalty.orientation_relaxation[self.vars.current_frame] == 1.0:
            params_log_tmp.append(self.penalty.orientation_relaxation[self.vars.current_frame])
        else:
            params_log_tmp.append(1.0-self.penalty.orientation_relaxation[self.vars.current_frame])
        with open(PARAM_RT_DIR_BASE+'crappy_motion_0_rev_mod36.csv', 'ab') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(params_log_tmp)

    def __construct_constraint_dict(self, constraints):
        constraint_dicts = []
        for c in constraints:
            d = {
                'type': c.constraintType(),
                'fun': c.func,
                'args': (self.vars,)
            }
            constraint_dicts.append(d)

        return tuple(constraint_dicts)