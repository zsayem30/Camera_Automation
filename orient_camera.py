#!/usr/bin/env python

import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
import time

### 28.3 cm to 0.283 m
def orient_camera(psm1, psm3, PSM1_camera_disp):
    
    T_1P_frame = psm1.measured_cp()
    T_1P_matrix = pm.toMatrix(T_1P_frame)

    ## Create desired pick up camera matrix

    T_D1_matrix = np.eye(4)
    T_D1_matrix[0:3, 0] = T_1P_matrix[0:3, 2]
    T_D1_matrix[0:3, 1] = T_1P_matrix[0:3, 0]
    T_D1_matrix[0:3, 2] = T_1P_matrix[0:3, 1]

    ## create position displacement 3X1 matrix
    T_D1_matrix[1, 3] = -PSM1_camera_disp

    ## concatenating matrix transformations
    T_DP_matrix = T_D1_matrix * T_1P_matrix

    T_PB_frame = psm3.measured_cp()
    T_PB_matrix = pm.toMatrix(T_PB_frame)
    T_DB_matrix = T_DP_matrix * T_PB_matrix

    psm3_goal = pm.fromMatrix(T_DB_matrix)
    psm3.move_cp(psm3_goal).wait()

def ready_arm(arm):
    arm.enable()
    arm.home()

def move_origin(arm, dx):

    origin = arm.measured_cp()
    dv = PyKDL.Vector(dx, dx, dx)
    origin.p = origin.p + dv
    arm.move_cp(origin).wait()

def orient_PSM3toPSM1(psm1, psm3):
    T_1P_frame = psm1.measured_cp()
    # print(T_1P_frame)
    T_1P_matrix = pm.toMatrix(T_1P_frame)
    C_1P = T_1P_matrix[0:3, 0:3]
    print("PSM3")
    print(C_1P)
    T_PB_frame = psm3.measured_cp()
    T_PB_matrix = pm.toMatrix(T_PB_frame)        
    C_PB = T_PB_matrix[0:3, 0:3]
    
    print("PSM1")
    print(C_PB)
    print("PSM3 * PSM1")
    C_1B = np.matmul(C_1P, C_PB)
    print(np.transpose(C_1B))

    print("Actual Orientation")
    T_1B_matrix = np.eye(4)
    T_1B_matrix[0:3, 0:3] = C_1B
    T_1B_matrix[0:3, 3] = T_PB_matrix[0:3, 3]

    T_1B_frame = pm.fromMatrix(T_1B_matrix)

    return np.transpose(C_1B)


    # time.sleep(2)
    # psm3.move_cp(T_1B_frame).wait()
    # print(psm3.measured_cp().M)
if __name__ == '__main__':
    # rospy.init_node('dvrk_origin_test', anonymous=True)
    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")

    ready_arm(psm1)
    ready_arm(psm3)

    orient_camera(psm1, psm3, 0.15)



    
