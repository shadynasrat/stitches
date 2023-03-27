#!/usr/bin/python

import cv2
import numpy as np
import keyboard
import json
import pyrealsense2 as rs

def stitch():
    with open('data.json', 'r') as f:
        data = json.load(f)

    lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4 = data

    # Configure depth and color streams...
    # ...from Camera 1
    width  = 720
    height = 1280

    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('108522070565')
    config_1.enable_stream(rs.stream.color, height, width, rs.format.bgr8, 30)
    # ...from Camera 2
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device('949122070270')
    config_2.enable_stream(rs.stream.color, height, width, rs.format.bgr8, 30)


    # Start streaming from both cameras
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)
    
    src_points = np.float32([[lx1, ly1], [lx2, ly2], [lx3, ly3], [lx4, ly4]])
    dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    M_1 = cv2.getPerspectiveTransform(src_points, dst_points)

    src_points = np.float32([[rx1, ry1], [rx2, ry2], [rx3, ry3], [rx4, ry4]])
    dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    M_2 = cv2.getPerspectiveTransform(src_points, dst_points)


    while 1:
        frames_1 = pipeline_1.wait_for_frames()
        color_frame_1 = frames_1.get_color_frame()
        left_image = np.asanyarray(color_frame_1.get_data())

        frames_2 = pipeline_2.wait_for_frames()
        color_frame_2 = frames_2.get_color_frame()
        right_image = np.asanyarray(color_frame_2.get_data())

        rotated_frame_1 = cv2.rotate(left_image, cv2.ROTATE_90_CLOCKWISE)
        rotated_frame_2 = cv2.rotate(right_image, cv2.ROTATE_90_CLOCKWISE)

        height, width = rotated_frame_1.shape[:2]

        warped_img1 = cv2.warpPerspective(rotated_frame_1, M_1, (width, height))
        warped_img2 = cv2.warpPerspective(rotated_frame_2, M_2, (width, height))

        result = np.concatenate((warped_img1, warped_img2), axis=1)
        cv2.imshow('result', result)

        key = cv2.waitKey(1)
        if 48 <= key <= 57:
            click_points_index = key - 48
        if key == 27:
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    stitch()
