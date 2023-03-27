#!/usr/bin/python

import cv2
import numpy as np
import keyboard
import json
import pyrealsense2 as rs


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


    # Start streaming from both cameras
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

    frame_select = 'r'
    point_select = 1

    ly1 = 0
    lx1 = 0
    ly2 = 0
    lx2 = width
    ly3 = height
    lx3 = width
    ly4 = height
    lx4 = 0
    ry1 = 0
    rx1 = 0
    ry2 = 0
    rx2 = width
    ry3 = height
    rx3 = width
    ry4 = height
    rx4 = 0

    while 1:
        frames_1 = pipeline_1.wait_for_frames()
        color_frame_1 = frames_1.get_color_frame()
        left_image = np.asanyarray(color_frame_1.get_data())

        frames_2 = pipeline_2.wait_for_frames()
        color_frame_2 = frames_2.get_color_frame()
        right_image = np.asanyarray(color_frame_2.get_data())

        frame_select, point_select, lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4 = update(frame_select, point_select, lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4)
        src_points = np.float32([[lx1, ly1], [lx2, ly2], [lx3, ly3], [lx4, ly4]])
        dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
        M_1 = cv2.getPerspectiveTransform(src_points, dst_points)

        src_points = np.float32([[rx1, ry1], [rx2, ry2], [rx3, ry3], [rx4, ry4]])
        dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
        M_2 = cv2.getPerspectiveTransform(src_points, dst_points)

        rotated_frame_1 = cv2.rotate(left_image, cv2.ROTATE_90_CLOCKWISE)
        rotated_frame_2 = cv2.rotate(right_image, cv2.ROTATE_90_CLOCKWISE)

        warped_img1 = cv2.warpPerspective(rotated_frame_1, M_1, (width, height+500))
        warped_img2 = cv2.warpPerspective(rotated_frame_2, M_2, (width+500, height+500))

        result = np.concatenate((warped_img1, warped_img2), axis=1)
        cv2.imshow('result', result)

        key = cv2.waitKey(1)
        if key == 27:
            break

    pipeline_1.stop()
    pipeline_2.stop()

    save_to_file(lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4)
    cv2.destroyAllWindows()

def update(frame_select, point_select, lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4):
    frame_select = update_selected_frame(frame_select)
    point_select = update_selected_point(point_select)
    if frame_select == 'l':
        if point_select == 1:
            ly1, lx1 = update_xy(ly1, lx1)
        if point_select == 2:
            ly2, lx2 = update_xy(ly2, lx2)
        if point_select == 3:
            ly3, lx3 = update_xy(ly3, lx3)
        if point_select == 4:
            ly4, lx4 = update_xy(ly4, lx4)
    elif frame_select == 'r':
        if point_select == 1:
            ry1, rx1 = update_xy(ry1, rx1)
        if point_select == 2:
            ry2, rx2 = update_xy(ry2, rx2)
        if point_select == 3:
            ry3, rx3 = update_xy(ry3, rx3)
        if point_select == 4:
            ry4, rx4 = update_xy(ry4, rx4)
    print("frame: ",frame_select, "point: ", point_select, lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4)
    return frame_select, point_select, lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4  

def update_selected_frame(frame_select):
    if keyboard.is_pressed('l'):
        return 'l'
    elif keyboard.is_pressed('r'):
        return 'r'
    else:
        return frame_select

def update_selected_point(point_select):
    if keyboard.is_pressed('1'):
        return 1
    elif keyboard.is_pressed('2'):
        return 2
    elif keyboard.is_pressed('3'):
        return 3
    elif keyboard.is_pressed('4'):
        return 4
    else:
        return point_select

def update_xy(y, x):
    if keyboard.is_pressed('up'):
        y += 1
    if keyboard.is_pressed('down'):
        y -= 1
    if keyboard.is_pressed('right'):
        x -= 1
    if keyboard.is_pressed('left'):
        x += 1
    return y, x

def save_to_file(lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4):
    ans = input("Save callibration to file?(y/n)")
    print(ans)
    if ans.lower() == 'y':
        data = lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4

        # Save the dictionary to a JSON file
        with open('data.json', 'w') as f:
            json.dump(data, f)
        
        print("Saved succesfully")
    else:
        print("Discard")

if __name__ == "__main__":
    stitch()

#[0, 172, 720, 0, 720, 1280, -4, 1088, 17, 22, 699, 140, 728, 1158, 70, 1281]
#[0, 72, 700, -55, 720, 1276, 0, 1148, -3, -58, 672, 76, 691, 1140, 56, 1264]

