"""
Author: David Valencia
Date: 03/ 05 /2022
Update: 10/ 05 /2022

Describer:
            Main Reinforcement Learning file for robot gripper test bed
            This script is the core of the project. The state-space, action, reward calculation and data from
            camera and motors all are generated/read here

            state-space  = joint1 position(each arm),end effector position(each arm),goal position and cube position
            action-space = action vector size = 4 (one action for each motor)

            To run this file, please use the run_env.py file

            if close_env funct is called:
                1) DISABLE Torque for each motor
                2) Close the USB port
"""

import random
from threading import Thread
from motor_ultilities import *
from vision_utilities import *


class RL_ENV:

    def __init__(self):
        self.target_in_pixel_x = 260
        self.target_in_pixel_y = 150

        self.position_joint_1_arm_1 = 0
        self.position_end_arm_1     = 0

        self.position_joint_1_arm_2 = 0
        self.position_end_arm_2     = 0

        self.goal_position = 0
        self.cube_position = 0

    def reset_env(self):
        # move the robot to home position:
        id_1_dxl_home_position = 511
        id_2_dxl_home_position = 511
        id_3_dxl_home_position = 511
        id_4_dxl_home_position = 511

        print("Sending Robot to Home Position")
        move_motor_step(id_1_dxl_home_position, id_2_dxl_home_position, id_3_dxl_home_position, id_4_dxl_home_position)

        print("New Goal point generated")
        # generate a new goal position value in PIXELS between the operation area
        self.target_in_pixel_x = random.randint(150, 375)
        self.target_in_pixel_y = random.randint(150, 220)


    def forward_kinematic(self, tetha1, tetha2, l1, l2, d_x, d_y):
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

    def state_space_funct(self):
        # generate the observation state
        # all  w.r.t the reference frame

        # See the readme file and draw to understand these values
        l_1 = 6.0  # size of link one, cm dimension
        l_2 = 4.0  # size of link two, cm dimension

        d_x_1 = 9.0  # x displacement of arm 1 wrt the reference frame
        d_x_2 = 2.0  # x displacement of arm 2 wrt the reference frame
        d_y = 4.5  # y displacement of arm 1 and arm2 wrt the reference frame

        tetha_1_arm_1, tetha_2_arm_1, tetha_1_arm_2, tetha_2_arm_2 = get_angles()

        x_joint_1_arm_1, y_joint_1_arm_1, x_end_arm1, y_end_arm1 = self.forward_kinematic(tetha_1_arm_1, tetha_2_arm_1,
                                                                                          l_1, l_2, d_x_1, d_y)

        x_joint_1_arm_2, y_joint_1_arm_2, x_end_arm2, y_end_arm2 = self.forward_kinematic(tetha_1_arm_2, tetha_2_arm_2,
                                                                                          l_1, l_2, d_x_2, d_y)

        self.position_joint_1_arm_1 = (x_joint_1_arm_1, y_joint_1_arm_1)
        self.position_end_arm_1     = (x_end_arm1, y_end_arm1)

        self.position_joint_1_arm_2 = (x_joint_1_arm_2, y_joint_1_arm_2)
        self.position_end_arm_2     = (x_end_arm2, y_end_arm2)

        self.goal_position = calculate_transformation_target(self.target_in_pixel_x, self.target_in_pixel_y)
        self.cube_position = calculate_cube_position()

        return self.position_joint_1_arm_1, self.position_end_arm_1, \
               self.position_joint_1_arm_2, self.position_end_arm_2, \
               tuple(self.goal_position[0][0]), tuple(self.cube_position[0])  # tuple here is to avoid brackets/arrays
                                                                              # and have all the data in the same style

    def generate_sample_act(self):
        # generated random action(step values to be passed as goal positions)
        # Be very careful in the range to avoid arm collisions
        act_m1 = random.randint(300, 400)
        act_m2 = random.randint(300, 400)
        act_m3 = random.randint(300, 400)
        act_m4 = random.randint(300, 400)
        return [act_m1, act_m2, act_m3, act_m4]


    def env_step(self, actions):
        id_1_dxl_goal_position = actions[0]
        id_2_dxl_goal_position = actions[1]
        id_3_dxl_goal_position = actions[2]
        id_4_dxl_goal_position = actions[3]

        move_motor_step(id_1_dxl_goal_position, id_2_dxl_goal_position, id_3_dxl_goal_position, id_4_dxl_goal_position)

    def calculate_reward(self):
        distance_cube_goal = calculate_distance(self.cube_position, self.goal_position)

        if distance_cube_goal <= 0.3:  # cm
            done = True
            reward_d = 100
        else:
            done = False
            reward_d = (1/distance_cube_goal) * 10

        return reward_d, done

    def render(self):
        t1 = Thread(target=self.run_video_d)  #
        t1.start()


    def run_video_d(self):
        font = cv2.FONT_HERSHEY_SIMPLEX
        while cam.isOpened():
            ret, image = cam.read()
            if ret:
                    target = (self.target_in_pixel_x, self.target_in_pixel_y)
                    text_in_target = (self.target_in_pixel_x - 30, self.target_in_pixel_y+3)
                    cv2.circle(image, target, 25, (0, 0, 250), -1)
                    cv2.putText(image, 'Target Point', text_in_target, font, 0.3, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow('camera frame', image)
            if cv2.waitKey(1) == ord('q'):
                break

    def close_env(self):
        motor_terminate()

