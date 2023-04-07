import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RVIZ_VIEWER:
    def __init__(self):
        self.img = None
        rospy.init_node("rviz")
        self.left_topic = "/camera/color/image_raw"
        rospy.Subscriber(self.left_topic, Image, self.callback_left)
        self.bridge = CvBridge()

    def callback_left(self, data):
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def get_screen(self):
        while not rospy.is_shutdown():
            if self.img is not None:
                cv2.imshow("RVIZ VIEWER", self.img)
            if cv2.waitKey(1) == 27:
                break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    custom_view = RVIZ_VIEWER()
    custom_view.get_screen()