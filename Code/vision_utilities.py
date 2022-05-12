"""
Author: David Valencia
Date: 10/ 05 /2022

Description:
             functions associated with camera and opencv to obtain the position of aruco markers, make transformations
              and calculate the distance

              id of marker in reference frame = 10
"""

import os
import cv2
import numpy as np

from os.path import expanduser
home = expanduser("~")
# These values come after calibrating the camera
matrix_dist_path = os.path.join(home, '/manipulator_learning_testbed/camera_calibration_method') # make sure use the correct path

matrix      = np.loadtxt(open(os.path.join(matrix_dist_path, "matrix.txt"), 'rb'))
distortion  = np.loadtxt(open(os.path.join(matrix_dist_path, "distortion.txt"), "rb"))

# Aruco Dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

arucoParams   = cv2.aruco.DetectorParameters_create()

markerSizeInM = 0.025  # size of the aruco marker
cam = cv2.VideoCapture(0)  # open the camera


def get_camera_image():
    ret, frame = cam.read()
    if ret:
        return frame


def calculate_transformation_target(target_x_pix, target_y_pix):
    image = get_camera_image()

    # Detect Aruco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

    if len(corners) > 0:  # if there is at least one id on the screen

        # calculate the size of the aruco mark
        s = np.abs(corners[0][0][0][0] - corners[0][0][3][0])  # dimension side of the square
        s = s / 2
        a = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]])  # support matrix to create the fake corners

        # virtually create corners around the target point as a "fake marker"
        target_marker_corners = [a * s + (target_x_pix, target_y_pix)]
        target_marker_corners = [np.array(target_marker_corners, dtype="float32")]

        # rotation and translation of virtual target marker w.r.t camera
        target_rvec, target_tvec, _ = cv2.aruco.estimatePoseSingleMarkers(target_marker_corners, markerSizeInM,
                                                                          matrix, distortion)

        # rotation and translation of real markers w.r.t camera
        reference_rvec, refence_tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,
                                                                              matrix, distortion)

        # all these values w.r.t camera frame
        mark_1 = refence_tvec[0]  # reference frame marker
        target = target_tvec      # target virtual marker

        # coordinates w.r.t. reference marker
        position_target = mark_1 - target  # this unit is meters
        position_target = position_target * [-1, 1, 1]  # I need this for the x-axis to match with the reference frame
        position_target_cm = position_target * 100  # this unit is CM


        # another way to do the same calculation
        '''
        rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM, matrix, distortion) # rotation and traslation w.r.t camera of aruco markers i.e origen
        
        z = tvec[0,0,2]

        cx = matrix[0,2]
        fx = matrix[0,0]
        cy = matrix[1,2]
        fy = matrix[1,1]

        px = (target_x_pix - cx) / fx
        py = (target_y_pix - cy) / fy
        
        px = px * z
        py = py * z
        pz = z

        # coordinates w.r.t. camera frame
        reference_marker_wrt_camera = tvec[0] # reference frame marker
        target_point_wrt_camera     = (px, py, pz)

        # coordinates w.r.t. reference marker
        position_target = reference_marker_wrt_camera - target_point_wrt_camera  # this unit is meters
        position_target_cm = position_target * 100
        print(position_target_cm)
        '''
        return position_target_cm


def calculate_cube_position():

    image = get_camera_image()

    # Detect Aruco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

    if len(corners) > 1:  # if there are at least two ids on the screen

        # rotation and translation w.r.t camera of aruco markers i.e origen frame and cube
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM, matrix, distortion)

        # all these values w.r.t camera frame
        mark_1 = tvec[0]  # reference frame marker
        mark_2 = tvec[1]  # cube frame marker

        # coordinates w.r.t. reference marker
        position_cube = mark_1 - mark_2  # this unit is meters
        position_cube = position_cube * [-1, 1, 1]  # I need this for the x-axis to match with the reference frame
        position_cube_cm = position_cube * 100

        return position_cube_cm


def calculate_distance(cube, goal_position):
    # distance between cube and target point

    #dist_cube_target_point = np.linalg.norm(cube - goal_position)
    dist_cube_target_point = np.linalg.norm(cube[0][:-1] - goal_position[0][0][:-1])  # I am not considering z axis here
    return dist_cube_target_point


def close_camera_set():
    cam.release()
    cv2.destroyAllWindows()


