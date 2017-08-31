__author__ = 'drakita'


from urdf_parser_py.urdf import URDF
from Spacetime.arm import *
from colors import *
import kdl_parser_py.urdf as pyurdf
import PyKDL as kdl

'''
NOTE:
These functions require kdl_parser_py and urdf_parser_py to be installed

commands to install these:
>> sudo apt-get install ros-[your ros distro]-urdfdom-py

>> sudo apt-get install ros-[your ros distro]-kdl-parser-py
>> sudo apt-get install ros-[your ros distro]-kdl-conversions

'''

def urdf_load(urdfString, startJoint, endJoint, full_joint_list, fixed_ee_joint = None, Debug=False):
    '''
    Takes in a urdf file and parses that into different object representations of the robot arm

    :param urdfString: (string) file path to the urdf file.  example: < 'mico.urdf' >
    :param startJoint: (string) name of the starting joint in the chain
    :param endJoint: (string) name of the final joint in the chain
        NOTE: this is the final ROTATING joint in the chain, for a fixed joint on the end effector, add this as the fixed joint!
    :param fixed_ee_joint (string) name of the fixed joint after end joint in the chain.  This is read in just to get a final
        displacement on the chain, i.e. usually just to add in an extra displacement offset for the end effector
    :return: returns the robot parsed object from urdf_parser, Mike's "arm" version of the robot arm, as well as the kdl tree
    '''
    print(urdfString)
    print("=================================================================================================================")
    urdf_robot = URDF.from_xml_file(urdfString)
    (ok, kdl_tree) = pyurdf.treeFromFile(urdfString)
    if not (startJoint == '' or endJoint == ''):
        chain = kdl_tree.getChain(startJoint, endJoint)
    if full_joint_list == ():
        arm = convertToArm(urdf_robot, startJoint, endJoint, fixed_ee_joint, Debug=Debug)
    else:
        arm = convertToArmJointList(urdf_robot, full_joint_list, fixed_ee_joint, Debug=Debug)

    if Debug:
        o = open('out', 'w')
        o.write(str(urdf_robot))

    return urdf_robot, arm, kdl_tree

def convertToArmJointList(urdf_robot, full_joint_list, fixedJoint, Debug=False):
    if urdf_robot == None:
        raise ValueError('Incorrect Argument in convertToArm.  urdf_robot is None type.')

    joints = urdf_robot.joints

    name = urdf_robot.name
    axes = []
    offset = []
    displacements = []
    rotOffsets = []
    firstPass = True

    for j in full_joint_list:
        for js in joints:
            if js.name == j:
                if firstPass:
                    axes.append(toAxisLetter(js.axis))
                    offset = tuple(js.origin.xyz)
                    rotOffsets.append(tuple(js.origin.rpy))
                    firstPass = False
                else:
                    axes.append(toAxisLetter(js.axis))
                    displacements.append(tuple(js.origin.xyz))
                    rotOffsets.append(tuple(js.origin.rpy))

        # add any additional joints in the chain listed after the end joint
    if not fixedJoint == None:
        currJoint = []
        for j in joints:
            if j.name == fixedJoint:
                currJoint = j
                displacements.append(tuple(currJoint.origin.xyz))
        if currJoint == []:
            print bcolors.FAIL + 'fixed_ee_joint: {} not found!'.format(fixedJoint) + bcolors.ENDC
            raise Exception('Invalid fixed_ee_joint.  Exiting.')


    numDOF = len(axes)
    rotOffsets = rotOffsets[0:numDOF]

    if Debug:
        outStr = 'name:\n {} \n axes:\n {} \n displacements:\n {} \n ' \
                 'rotOffsets:\n {} \n offset:\n {} offset'.format(name, tuple(axes), displacements, rotOffsets, offset)

        print outStr

    return Arm(tuple(axes), displacements, rotOffsets, offset, name)



def convertToArm(urdf_robot, startJoint, endJoint, fixedJoint, Debug=False):
    '''
    This function parses the (axes, offset, displacements, rotOffsets) in order to return an Arm
    from mg's spacetime code

    :param urdf_robot: urdf_robot object returned from URDF.from_xml_file
    :param startJoint: start joint string, e.g. <'shoulder_pan_joint'> for UR5
    :param endJoint:  end joint string, e.g. <'ee_fixed_joint'> for UR5
    :return: Arm object
    '''

    if urdf_robot == None:
        raise ValueError('Incorrect Argument in convertToArm.  urdf_robot is None type.')

    joints = urdf_robot.joints

    s = []
    e = []
    for j in joints:
        if j.name == startJoint:
            s = j
        if j.name == endJoint:
            e = j
    if s == []:
        print bcolors.FAIL + 'startJoint: {} not found in joint list!  Please check to make sure startJoint is a joint found in' \
                         'the URDF and is spelled correctly'.format(startJoint) + bcolors.ENDC
        raise ValueError('Invalid Value.  Exiting.')
    if e == []:
        print bcolors.FAIL + 'endJoint: {} not found in joint list!  Please check to make sure endJoint is a joint found in' \
                         'the URDF and is spelled correctly'.format(endJoint).format(startJoint) + bcolors.ENDC
        raise ValueError('Invalid Value.  Exiting.')

    name = urdf_robot.name
    axes = []
    offset = []
    displacements = []
    rotOffsets = []

    currJoint = s
    axes.append(toAxisLetter(currJoint.axis))
    offset = tuple(currJoint.origin.xyz)
    rotOffsets.append(tuple(currJoint.origin.rpy))

    currJoint = findNextJoint(joints, currJoint.child)

    while True:
        axes.append(toAxisLetter(currJoint.axis))
        displacements.append(tuple(currJoint.origin.xyz))
        rotOffsets.append(tuple(currJoint.origin.rpy))
        if currJoint.name == endJoint:
            break
        currJoint = findNextJoint(joints, currJoint.child)

    # add any additional joints in the chain listed after the end joint
    if not fixedJoint == None:
        currJoint = []
        for j in joints:
            if j.name == fixedJoint:
                currJoint = j
                displacements.append(tuple(currJoint.origin.xyz))
        if currJoint == []:
            print bcolors.FAIL + 'fixed_ee_joint: {} not found!'.format(fixedJoint) + bcolors.ENDC
            raise Exception('Invalid fixed_ee_joint.  Exiting.')



    numDOF = len(axes)
    rotOffsets = rotOffsets[0:numDOF]

    if Debug:
        outStr = 'name:\n {} \n axes:\n {} \n displacements:\n {} \n ' \
                 'rotOffsets:\n {} \n offset:\n {} offset'.format(name, tuple(axes), displacements, rotOffsets, offset)

        print outStr

    return Arm(tuple(axes), displacements, rotOffsets, offset, name)

def toAxisLetter(ax):
    if ax == None:
        return ''
    ax_val = ''
    if ax[0] == 1:
        ax_val = 'x'
    elif ax[1] == 1:
        ax_val = 'y'
    elif ax[2] == 1:
        ax_val = 'z'
    return ax_val

def findNextJoint(joints, child):
    for j in joints:
        if j.parent == child:
            return j
    print bcolors.FAIL + 'joint with matching parent link: {} not found!'.format(child) + bcolors.ENDC
    raise Exception('Invalid joint chain.  Exiting.')




