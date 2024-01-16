#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from pynput import keyboard
from Scripts.msg import TargetInfo
from Scripts.srv import UpdateState, ItemPositionFOV


class RhInterface:

    def __init__(self):

        rospy.init_node("rh_interface")

        self.bridge = CvBridge()
        self.rate = rospy.Rate(5)

        self.image = None
        self.width = None
        self.height = None
        self.center_x = None
        self.center_y = None
        self.target_detected = False
        self.state = 0

        # Subscribers
        rospy.Subscriber(
            '/chest_cam/camera/color/image_raw',
            Image,
            self.image_callback,
        ),

        rospy.Subscriber(
            '/workspace_cam/target_position_in_frame',
            Float32MultiArray,
            self.target_position_frame,
        ),

        self.image_pub = rospy.Publisher(
            "/chest_cam/remote_interface",
            Image,
            queue_size=1,
        )

    def image_callback(self, data):

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

    def target_position_frame(self, message):

        self.center_x = int(message.data[0])
        self.center_y = int(message.data[1])

        if (self.center_x == 0) and (self.center_y == 0):
            self.target_detected = False
        else:
            self.target_detected = True

    def main_loop(self):

        if self.image is not None:
            self.image = self.image.copy()

            if self.target_detected:
                cv2.circle(
                    self.image, (self.center_x, self.center_y),
                    radius=30,
                    color=(0, 255, 0),
                    thickness=2
                )

            image = Image()
            image.data = np.array(cv2.imencode('.png', self.image)[1]).tobytes()

            self.image_pub.publish(image)


if __name__ == "__main__":

    remote_interface = RhInterface()

    while not rospy.is_shutdown():

        remote_interface.main_loop()
        remote_interface.rate.sleep()
