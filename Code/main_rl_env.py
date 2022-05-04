"""
Author: David Valencia
Date: 03/ 05 /2022

Describer:
            Main Reinforcement Learning file for robot gripper test bed
            This script is the core of the project. The state-space, action, reward calculation and data from
            camera and motors all are generated/read here

            state-space = joint1 position (each arm),end effector position (each arm), target position and cube position
            action-space = currently just random actions are generated

            If the number of step in each episode finished the environment will reset so the robot will
            move to the home position

            To run this file, please use the run_env.py file

            if motor_terminate funct is called:
                1) DISABLE Torque for each motor
                2) Close the USB port
"""

import random
import numpy as np
from motor_initialization import *


def read_servo_position(motor_id):
    """
    :param motor_id:
    :return: position of the motor (step value)
    """
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return dxl_present_position


def get_angles():
    # Arm 1
    # position in steps values
    pos_m1_arm_1 = read_servo_position(DXL_ID_1)
    pos_m2_arm_1 = read_servo_position(DXL_ID_2)

    # Arm 2
    # position in steps values
    pos_m3_arm_2 = read_servo_position(DXL_ID_3)
    pos_m4_arm_2 = read_servo_position(DXL_ID_4)

    # Values in degrees
    tetha_1_arm_1 = pos_m1_arm_1 * 0.29326
    tetha_2_arm_1 = pos_m2_arm_1 * 0.29326

    tetha_1_arm_2 = pos_m3_arm_2 * 0.29326
    tetha_2_arm_2 = pos_m4_arm_2 * 0.29326

    # IMPORTANT, need these values in order to match the equations
    tetha_1_arm_1 = tetha_1_arm_1 - 60
    tetha_1_arm_2 = 240 - tetha_1_arm_2

    tetha_2_arm_1 = 150 - tetha_2_arm_1
    tetha_2_arm_2 = tetha_2_arm_2 - 150

    return tetha_1_arm_1, tetha_2_arm_1, tetha_1_arm_2, tetha_2_arm_2


def forward_kinematic(tetha1, tetha2, l1, l2, d_x, d_y):
    tetha1_rad = np.deg2rad(tetha1)
    tetha2_rad = np.deg2rad(tetha2)

    x_1 = l1 * np.cos(tetha1_rad)
    y_1 = l1 * np.sin(tetha1_rad)

    x_2 = x_1 + l2 * np.cos(tetha1_rad + tetha2_rad)
    y_2 = y_1 + l2 * np.sin(tetha1_rad + tetha2_rad)

    # joint 1 position w.r.t the reference frame
    x_1_r = x_1 + d_x
    y_1_r = y_1 + d_y

    # end-effector position w.r.t the reference frame
    x_2_r = d_x + x_2
    y_2_r = d_y + y_2

    return x_1_r, y_1_r, x_2_r, y_2_r


def target_position():
    pass


def cube_position():
    pass


def state_space_funct():
    """
    :return: the state space w.r.t the reference frame
    """

    # See the read me file and draw to understand these values
    l_1 = 6.0  # size of link one, cm dimension
    l_2 = 4.0  # size of link two, cm dimension

    d_x_1 = 9.0  # x displacement of arm 1 wrt the reference frame
    d_x_2 = 2.5  # x displacement of arm 2 wrt the reference frame
    d_y = 6.5  # y displacement of arm 1 and arm2 wrt the reference frame

    tetha_1_arm_1, tetha_2_arm_1, tetha_1_arm_2, tetha_2_arm_2 = get_angles()

    x_joint_1_arm_1, y_joint_1_arm_1, x_end_arm1, y_end_arm1 = forward_kinematic(tetha_1_arm_1, tetha_2_arm_1, l_1, l_2, d_x_1, d_y)
    x_joint_1_arm_2, y_joint_1_arm_2, x_end_arm2, y_end_arm2 = forward_kinematic(tetha_1_arm_2, tetha_2_arm_2, l_1, l_2, d_x_2, d_y)

    # target position here
    # cube position here

    return x_joint_1_arm_1, y_joint_1_arm_1, x_end_arm1, y_end_arm1,  \
           x_joint_1_arm_2, y_joint_1_arm_2, x_end_arm2, y_end_arm2


def calculate_reward():
    pass


def generate_random_act():
    # generated random action(step values to be passed as goal positions)
    # Be very careful in the range to avoid arm collisions
    act_m1 = random.randint(300, 400)
    act_m2 = random.randint(300, 400)
    act_m3 = random.randint(300, 400)
    act_m4 = random.randint(300, 400)
    return [act_m1, act_m2, act_m3, act_m4]


def env_step():

    actions = generate_random_act()

    id_1_dxl_goal_position  = actions[0]
    id_2_dxl_goal_position  = actions[1]
    id_3_dxl_goal_position  = actions[2]
    id_4_dxl_goal_position  = actions[3]

    param_goal_position_1 = [DXL_LOBYTE(id_1_dxl_goal_position), DXL_HIBYTE(id_1_dxl_goal_position)]
    param_goal_position_2 = [DXL_LOBYTE(id_2_dxl_goal_position), DXL_HIBYTE(id_2_dxl_goal_position)]
    param_goal_position_3 = [DXL_LOBYTE(id_3_dxl_goal_position), DXL_HIBYTE(id_3_dxl_goal_position)]
    param_goal_position_4 = [DXL_LOBYTE(id_4_dxl_goal_position), DXL_HIBYTE(id_4_dxl_goal_position)]

    move_motor_step(param_goal_position_1, param_goal_position_2, param_goal_position_3, param_goal_position_4)


def move_motor_step(param_goal_position_1, param_goal_position_2, param_goal_position_3, param_goal_position_4):
    # This function move the motor i.e. take the actions

    # --- Add the goal position value to the GroupSync, motor ID1 ----
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID_1, param_goal_position_1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID_1)
        quit()

    # --- Add the goal position value to the GroupSync, motor ID2 ----
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID_2, param_goal_position_2)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID_2)
        quit()

    # --- Add the goal position value to the GroupSync, motor ID3 ----
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID_3, param_goal_position_3)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID_3)
        quit()

    # --- Add the goal position value to the GroupSync, motor ID4 ----
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID_4, param_goal_position_4)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID_4)
        quit()

    # ---- Transmits packet (goal positions) to the motors
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()


def reset_env():
    # move the robot to home postion:
    id_1_dxl_home_position  = 511
    id_2_dxl_home_position  = 511
    id_3_dxl_home_position  = 511
    id_4_dxl_home_position  = 511

    param_home_position_1 = [DXL_LOBYTE(id_1_dxl_home_position), DXL_HIBYTE(id_1_dxl_home_position)]
    param_home_position_2 = [DXL_LOBYTE(id_2_dxl_home_position), DXL_HIBYTE(id_2_dxl_home_position)]
    param_home_position_3 = [DXL_LOBYTE(id_3_dxl_home_position), DXL_HIBYTE(id_3_dxl_home_position)]
    param_home_position_4 = [DXL_LOBYTE(id_4_dxl_home_position), DXL_HIBYTE(id_4_dxl_home_position)]

    move_motor_step(param_home_position_1, param_home_position_2, param_home_position_3, param_home_position_4)


def motor_terminate():
    # Disable communication and close the port
    # ---------------Disable Torque for each motor-----------------------
    # Enable Dynamixel_1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disable torque" % DXL_ID_1)

    # Enable Dynamixel_2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disable torque" % DXL_ID_2)

    # Enable Dynamixel_3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disable torque" % DXL_ID_3)

    # Enable Dynamixel_4 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_4, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disable torque" % DXL_ID_4)
    # Close port
    portHandler.closePort()
    print("Succeeded to close the USB port ")
