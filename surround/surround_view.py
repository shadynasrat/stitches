#!/usr/bin/python

import cv2
import numpy as np
import keyboard
import json
import pyrealsense2 as rs

class Viewer:
    def __init__(self):
        # Configure depth and color streams...
        self.width  = 720
        self.height = 1280
        # ...from Camera 1
        self.pipeline_1 = rs.pipeline()
        config_1 = rs.config()
        config_1.enable_device('108522070565')
        config_1.enable_stream(rs.stream.color, self.height, self.width, rs.format.bgr8, 30)
        # ...from Camera 2
        self.pipeline_2 = rs.pipeline()
        config_2 = rs.config()
        config_2.enable_device('949122070270')
        config_2.enable_stream(rs.stream.color, self.height, self.width, rs.format.bgr8, 30)
        # ...from Camera 3
        self.pipeline_3 = rs.pipeline()
        config_3 = rs.config()
        config_3.enable_device('935422071618')
        config_3.enable_stream(rs.stream.color, self.height, self.width, rs.format.bgr8, 30)
        # ...from Camera 2
        self.pipeline_4 = rs.pipeline()
        config_4 = rs.config()
        config_4.enable_device('109122071346')
        config_4.enable_stream(rs.stream.color, self.height, self.width, rs.format.bgr8, 30)

        # Start streaming from both cameras
        self.pipeline_1.start(config_1)
        self.pipeline_2.start(config_2)
        self.pipeline_3.start(config_3)
        self.pipeline_4.start(config_4)

        self.frame_select = 'r'
        self.point_select = 1

        self.load_data = True

        self.Matrix = np.zeros((5,4,2), dtype=int)
        self.pointer = [0,0]

        if self.load_data:
            with open('data.json', 'r') as f:
                data = json.load(f)

            self.Matrix = np.array(data)


        self.Matrix[:,1,0] = self.width
        self.Matrix[:,2,1] = self.height
        self.Matrix[:,2,0] = self.width
        self.Matrix[:,3,1] = self.height

        self.dst_points = np.float32([[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]])

    def stitch(self):
        while 1:
            frames_1 = self.pipeline_1.wait_for_frames()
            color_frame_1 = frames_1.get_color_frame()
            front_left_image = np.asanyarray(color_frame_1.get_data())

            frames_2 = self.pipeline_2.wait_for_frames()
            color_frame_2 = frames_2.get_color_frame()
            front_right_image = np.asanyarray(color_frame_2.get_data())

            frames_3 = self.pipeline_3.wait_for_frames()
            color_frame_3 = frames_3.get_color_frame()
            left_image = np.asanyarray(color_frame_3.get_data())

            frames_4 = self.pipeline_4.wait_for_frames()
            color_frame_4 = frames_4.get_color_frame()
            right_image = np.asanyarray(color_frame_4.get_data())

            self.get_key()
            self.update_Matrix()
            
            src_points = np.float32(self.Matrix[0])
            M_1 = cv2.getPerspectiveTransform(src_points, self.dst_points)

            src_points = np.float32(self.Matrix[1])
            M_2 = cv2.getPerspectiveTransform(src_points, self.dst_points)

            src_points = np.float32(self.Matrix[2])
            M_3 = cv2.getPerspectiveTransform(src_points, self.dst_points)

            src_points = np.float32(self.Matrix[3])
            M_4 = cv2.getPerspectiveTransform(src_points, self.dst_points)

            src_points = np.float32(self.Matrix[4])
            M_5 = cv2.getPerspectiveTransform(src_points, self.dst_points)

            rotated_frame_1 = cv2.rotate(front_left_image, cv2.ROTATE_90_CLOCKWISE)
            rotated_frame_2 = cv2.rotate(front_right_image, cv2.ROTATE_90_CLOCKWISE)
            rotated_frame_3 = cv2.rotate(left_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            rotated_frame_4 = cv2.rotate(right_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            A1_image = cv2.imread('../imgs/A1.png')

            blank_image = np.zeros((self.height, 360, 3), np.uint8)
            warped_img1 = cv2.warpPerspective(rotated_frame_1, M_1, (self.width, self.height))
            warped_img2 = cv2.warpPerspective(rotated_frame_2, M_2, (self.width, self.height))
            warped_img3 = cv2.warpPerspective(rotated_frame_3, M_3, (self.width, self.height+560))
            warped_img4 = cv2.warpPerspective(rotated_frame_4, M_4, (self.width, self.height+560))
            warped_img5 = cv2.warpPerspective(A1_image, M_5, (self.width, A1_image.shape[0]))


            stacked_top = np.concatenate((blank_image, warped_img1, warped_img2, blank_image), axis=1)
            stacked_bot = np.concatenate((warped_img3, warped_img5, warped_img4), axis=1)
            result = np.vstack((stacked_top, stacked_bot))
            resized_result = cv2.resize(result, (1280, 800))
            cv2.imshow('result', resized_result)

            key = cv2.waitKey(1)
            if key == 27:
                break

        self.pipeline_1.stop()
        self.pipeline_2.stop()
        self.pipeline_3.stop()
        self.pipeline_4.stop()
        cv2.destroyAllWindows()
        self.save_to_file()

    def get_key(self):
        if keyboard.is_pressed('f1'):
            self.pointer[0] = 0
        elif keyboard.is_pressed('f2'):
            self.pointer[0] = 1
        elif keyboard.is_pressed('f3'):
            self.pointer[0] = 2
        elif keyboard.is_pressed('f4'):
            self.pointer[0] = 3
        elif keyboard.is_pressed('f5'):
            self.pointer[0] = 4
        elif keyboard.is_pressed('1'):
            self.pointer[1] = 0
        elif keyboard.is_pressed('2'):
            self.pointer[1] = 1
        elif keyboard.is_pressed('3'):
            self.pointer[1] = 2
        elif keyboard.is_pressed('4'):
            self.pointer[1] = 3

    def update_Matrix(self):
        if keyboard.is_pressed('up'):
            self.Matrix[self.pointer[0], self.pointer[1], 1] += 10               # y value
        if keyboard.is_pressed('down'):
            self.Matrix[self.pointer[0], self.pointer[1], 1] -= 10
        if keyboard.is_pressed('right'):
            self.Matrix[self.pointer[0], self.pointer[1], 0] -= 10               # x value
        if keyboard.is_pressed('left'):
            self.Matrix[self.pointer[0], self.pointer[1], 0] += 10

    def save_to_file(self):
        ans = input("Save callibration to file?(y/n)")
        print(ans)
        if ans.lower() == 'y':
            data = self.Matrix.tolist()

            with open('data.json', 'w') as f:
                json.dump(data, f)
            
            print("Saved succesfully")
        else:
            print("Discard")

view = Viewer()
view.stitch()