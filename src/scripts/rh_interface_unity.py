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

        self.rect_start_x = None
        self.rect_start_y = None
        self.rect_end_x = None
        self.rect_end_y = None

        self.button_start_x = None
        self.button_start_y = None

        self.button_one = None
        self.button_two = None
        self.font = cv2.FONT_HERSHEY_DUPLEX
        self.font_scale = 0.75
        self.font_thickness = 2

        # Define rectangle properties
        self.rect_position = (5, 5)
        self.rect_size = (215, 125)
        self.rect_color = (255, 255, 244)
        self.rect_border_color = (139, 139, 0)  # Red border color in BGR format
        self.rect_thickness = 2

        self.image = None
        self.width = None
        self.height = None
        self.center_x = None
        self.center_y = None
        self.target_detected = False
        self.rh_help = False

        self.month = None
        self.year = None
        self.marker_id = None
        self.medicine_name = None
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

        # rospy.Subscriber(
        #     '/target_identifier',
        #     TargetInfo,
        #     self.__target_identifier_callback,
        # ),

        self.image_pub = rospy.Publisher(
            "/chest_cam/remote_interface",
            CompressedImage,
            queue_size=1,
        )

        # Service provider
        self.remote_service = rospy.Service(
            '/local_request',
            UpdateState,
            self.local_help,
        )

        # self.remote_help_service = rospy.Service(
        #     '/remote_help_request_service',
        #     UpdateState,
        #     self.help_request,
        # )

        # Service subscriber
        # self.resume_task_service = rospy.ServiceProxy(
        #     '/resume_task',
        #     UpdateState,
        # )

        self.local_help_service = rospy.ServiceProxy(
            '/local_help_request_service',
            UpdateState,
        )

        # self.position_fov_service = rospy.ServiceProxy(
        #     '/calculate_world_position_service',
        #     ItemPositionFOV,
        # )

        # self.change_task_state_service = rospy.ServiceProxy(
        #     '/change_task_state_service',
        #     UpdateState,
        # )

    def local_help(self, request):

        self.rh_help = False
        self.local_help_service(request)
        return True

    def image_callback(self, data):

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

        self.image = self.image.copy()

        if self.target_detected:
            cv2.circle(
                self.image, (self.center_x, self.center_y),
                radius=30,
                color=(0, 255, 0),
                thickness=2
            )

        image = CompressedImage()
        # image.header = rospy.Time.now()
        image.format = "jpeg"
        image.data = np.array(cv2.imencode('.jpg', self.image)[1]).tobytes()

        self.image_pub.publish(image)

        cv2.waitKey(3)

    def target_position_frame(self, message):

        self.center_x = int(message.data[0])
        self.center_y = int(message.data[1])

        if (self.center_x == 0) and (self.center_y == 0):
            self.target_detected = False
        else:
            self.target_detected = True


if __name__ == "__main__":

    remote_interface = RhInterface()

    while not rospy.is_shutdown():
        remote_interface.rate.sleep()
