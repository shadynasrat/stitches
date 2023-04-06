import cv2
import numpy as np
import json
import pyrealsense2 as rs
import time

def stitch():
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
    # ...from Camera 3
    pipeline_3 = rs.pipeline()
    config_3 = rs.config()
    config_3.enable_device('935422071618')
    config_3.enable_stream(rs.stream.color, height, width, rs.format.bgr8, 30)
    # ...from Camera 2
    pipeline_4 = rs.pipeline()
    config_4 = rs.config()
    config_4.enable_device('109122071346')
    config_4.enable_stream(rs.stream.color, height, width, rs.format.bgr8, 30)


    # Start streaming from both cameras
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)
    pipeline_3.start(config_3)
    pipeline_4.start(config_4)

    timer = time.time()

    while 1:
        frames_1 = pipeline_1.wait_for_frames()
        color_frame_1 = frames_1.get_color_frame()
        left_image = np.asanyarray(color_frame_1.get_data())

        frames_2 = pipeline_2.wait_for_frames()
        color_frame_2 = frames_2.get_color_frame()
        right_image = np.asanyarray(color_frame_2.get_data())

        frames_3 = pipeline_3.wait_for_frames()
        color_frame_3 = frames_3.get_color_frame()
        right_ = np.asanyarray(color_frame_3.get_data())

        frames_4 = pipeline_4.wait_for_frames()
        color_frame_4 = frames_4.get_color_frame()
        left_ = np.asanyarray(color_frame_4.get_data())

        rotated_frame_1 = cv2.rotate(left_image, cv2.ROTATE_90_CLOCKWISE)
        rotated_frame_2 = cv2.rotate(right_image, cv2.ROTATE_90_CLOCKWISE)
        rotated_frame_3 = cv2.rotate(left_, cv2.ROTATE_90_CLOCKWISE)
        rotated_frame_3 = cv2.rotate(rotated_frame_3, cv2.ROTATE_90_CLOCKWISE)

        result = np.concatenate((rotated_frame_1, rotated_frame_2), axis=1)

        key = cv2.waitKey(1)
        cv2.imshow('front', result)
        cv2.imshow('left', rotated_frame_3)
        cv2.imshow('right', right_)

        if time.time() > timer + 10:
            print('saving')
            break


    cv2.imwrite('./imgs/front.jpg', result)
    cv2.imwrite('./imgs/left.jpg', rotated_frame_3)
    cv2.imwrite('./imgs/right.jpg', right_)

    pipeline_1.stop()
    pipeline_2.stop()
    pipeline_3.stop()
    pipeline_4.stop()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    stitch()

# rs-enumerate-devices | grep Serial
