#!/usr/bin/python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CustomView:
    def __init__(self):
        self.left_image = None
        self.right_image = None
        rospy.init_node("stitch_cameras")
        self.left_topic = "/camera1/color/image_raw"
        self.right_topic = "/camera1/color/image_raw"
        rospy.Subscriber(self.left_topic, Image, self.callback_left)
        rospy.Subscriber(self.right_topic, Image, self.callback_right)

    def callback_left(self, data):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        self.left_image = image

    def callback_right(self, data):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        self.right_image = image

    def custom_view(self):
        while not rospy.is_shutdown() and self.left_image is None and self.right_image is None:
            pass

        while not rospy.is_shutdown():
            if self.left_image is not None and self.right_image is not None:
                result = np.concatenate((self.left_image, self.right_image), axis=1)
                cv2.imshow("Result", result)
                if cv2.waitKey(1) == ord('q'):
                    break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    custom_view = CustomView()
    custom_view.custom_view()
