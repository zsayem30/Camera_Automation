#!/usr/bin/env python

import numpy as np
import dvrk
import PyKDL
import rospy

def move_origin(arm, dx):

    origin = arm.setpoint_cp()
    dv = PyKDL.Vector(dx, dx, dx)
    origin.p = origin.p + dv
    arm.move_cp(origin).wait()

def move_tool_to_origin(arm):

    goal = arm.setpoint_cp()
    goal.p = PyKDL.Vector(0.0, 0.0, 0.0)
    arm.move_cp(goal).wait()

def ready_arm(arm):
    arm.enable()
    arm.home()

def orient(arm):
    
    goal = arm.setpoint_cp()
    goal.M = PyKDL.Rotation(-0.48766937,  0.13329323, -0.86279285,   0.86353902,  0.21896398, -0.45426329, 0.12837034, -0.96658558, -0.22188593)
    arm.move_cp(goal).wait()

if __name__ == '__main__':
    rospy.init_node('dvrk_origin_test', anonymous=True)
    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")

    ready_arm(psm1)
    ready_arm(psm3)

    # move_origin(psm3, 0.05)
    # move_tool_to_origin(psm1)
    orient(psm3)
    # goal = psm1.setpoint_cp()
    # move 5cm in z direction
    # goal.p[1] += 0.05
    # psm1.move_cp(goal).wait()
