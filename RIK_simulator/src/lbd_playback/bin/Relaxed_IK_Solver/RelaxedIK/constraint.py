# modified by hwang
__author__ = 'drakita'

from abc import ABCMeta, abstractmethod

class Constraint:
    __metaclass__ = ABCMeta

    @abstractmethod
    def constraintType(self):
        'return eq for equality and ineq for inequality'
        return None

    @abstractmethod
    def func(self, x, *args):
        pass

class SingularityConstraint(Constraint):
    def constraintType(self):
        return 'ineq'

    def func(self, x, *args):
        vars = args[0]
        arm = vars.arm
        y = arm.getYoshikawaMeasure(x)
        lower_bound = 1e-18
        return y - lower_bound

class ElbowFlipAvoidenceConstraint(Constraint):
    def constraintType(self):
        return 'ineq'

    def func(self, x, *args):
        # returning joint angle value of elbow directly
        return x[2]

class SelfCollisionAvoidenceConstraint(Constraint):
    def constraintType(self):
        return 'ineq'

    def func(self, x, *args):
        # only consider distance between end effector and part of 
        # arm(between elbow and wrist 1)
        vars = args[0]
        arm = vars.arm
        frams = arm.getFrames(x)
        # let x0 be ee pos, x1 be the elbow joint, x2 be the wrist 1
        x0 = eePos = arm.getFrames(x)[0][-1]
        x1 = elbowPos = arm.getFrames(x)[0][2]
        x2 = wrist1Pos = arm.getFrames(x)[0][3]
        # the closed form solution to calculate distance between a point
        # to a line segment can be found at http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        dist = math.sqrt((np.linalg.norm(x1-x0)**2 * np.linalg.norm(x2-x1)**2 - np.dot(x1-x0, x2-x1) ** 2) / np.linalg.norm(x2-x1)**2)
        return (dist - 0.08)