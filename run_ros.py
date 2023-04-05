#!/usr/bin/python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json

def stitch():
    rospy.init_node("stitch_cameras")
    left_topic = "/camera1/color/image_raw"
    right_topic = "/camera2/depth/image_rect_raw"
    
    with open('data.json', 'r') as f:
        data = json.load(f)

    lx1, ly1, lx2, ly2, lx3, ly3, lx4, ly4, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4 = data

    rospy.Subscriber(left_topic, Image, callback_left)
    rospy.Subscriber(right_topic, Image, callback_right)
    pub = rospy.Publisher("stitched_camera/image_raw", Image, queue_size=10)

    width  = 720
    height = 1280

    src_points = np.float32([[lx1, ly1], [lx2, ly2], [lx3, ly3], [lx4, ly4]])
    dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    M_1 = cv2.getPerspectiveTransform(src_points, dst_points)

    src_points = np.float32([[rx1, ry1], [rx2, ry2], [rx3, ry3], [rx4, ry4]])
    dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    M_2 = cv2.getPerspectiveTransform(src_points, dst_points)

    while 1:
        rotated_frame_1 = cv2.rotate(left_image, cv2.ROTATE_90_CLOCKWISE)
        rotated_frame_2 = cv2.rotate(right_image, cv2.ROTATE_90_CLOCKWISE)

        height, width = rotated_frame_1.shape[:2]

        warped_img1 = cv2.warpPerspective(rotated_frame_1, M_1, (width, height))
        warped_img2 = cv2.warpPerspective(rotated_frame_2, M_2, (width, height))

        result = np.concatenate((warped_img1, warped_img2), axis=1)
        pub.publish(CvBridge().cv2_to_imgmsg(result, "bgr8"))


def callback_left(data):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    global left_image
    left_image = image

def callback_right(data):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    global right_image
    right_image = image
    
if __name__ == "__main__":
    stitch()
