#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32MultiArray
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import (Pose, Point, Quaternion)

from datetime import datetime
from holo_project.msg import TargetInfo
from Scripts.srv import UpdateState, ItemPositionFOV


class Ar:

    def __init__(self):

        rospy.init_node("interface")

        self.bridge = CvBridge()
        self.rate = rospy.Rate(5)

        self.image = None
        # Create an ArUco dictionary and detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.aruco_params,
        )
        self.marker_size = 0.02

        self.marker_id = None
        self.expiration = None
        self.center_x = 0
        self.center_y = 0

        self.robot_state = 0  # Running

        # Create dictionary to store position of detected markers
        self.__detected_markers_world = {}
        self.__detected_markers_corners = {}

        self.box_ids = [10, 11, 12, 13]

        # Camera calibration parameters (replace with your actual values)
        self.camera_matrix = np.array(
            [
                [909.1905517578125, 0.0, 644.7723999023438], 
                [0.0, 908.4768676757812, 377.5361633300781], 
                [0.0, 0.0, 1.0],
            ]
        )
        self.dist_coeffs = np.array([0, 0, 0, 0, 0])

        # Subscribers
        rospy.Subscriber(
            '/target_identifier',
            TargetInfo,
            self.__target_identifier_callback,
        ),

        # rospy.Subscriber(
        #     '/workspace_cam/camera/color/image_raw',
        #     Image,
        #     self.image_callback,
        # ),

        rospy.Subscriber(
            '/workspace_cam/camera/color/image_raw',
            Image,
            self.image_callback,
        ),

        # Publishers
        self.__target_camera_pub = rospy.Publisher(
            '/my_gen3/target_workspace_cam',
            Point,
            queue_size=1,
        )

        self.__target_position_frame_pub = rospy.Publisher(
            '/workspace_cam/target_position_in_frame',
            Float32MultiArray,
            queue_size=1,
        )

        # Service provider
        self.resume_task_service = rospy.Service(
            '/resume_task',
            UpdateState,
            self.resume_task,
        )

        # Service provider
        self.position_fov_service = rospy.Service(
            '/calculate_world_position_service',
            ItemPositionFOV,
            self.calculate_world_position,
        )

        # Service subscriber
        self.remote_help_service = rospy.ServiceProxy(
            '/remote_help_request_service',
            UpdateState,
        )

    def resume_task(self, request):

        print("Task resumed.")
        self.robot_state = 0
        self.remote_help_service(0)
        
        return True

    def apply_low_pass_filter(self, current_position, marker_id, alpha):

        if marker_id in self.__detected_markers_world:

            previous_position = self.__detected_markers_world[marker_id]

            if (np.linalg.norm(current_position - previous_position)) > 0.03:

                return previous_position
            else:
                return alpha * current_position + (
                    1 - alpha
                ) * previous_position
        else:
            return current_position

    def __target_identifier_callback(self, message):

        self.marker_id = message.id
        self.expiration = message.expiration

        if self.marker_id in self.box_ids:
            self.marker_size = 0.05
        else:
            self.marker_size = 0.02

    def image_callback(self, data):

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

        # self.image = cv2.rotate(self.image, cv2.ROTATE_90_CLOCKWISE)

        # Detect ArUco markers
        corners, ids, _ = self.aruco_detector.detectMarkers(self.image)

        if ids is not None:
            for i in range(len(ids)):

                self.__detected_markers_corners[ids[i][0]] = corners[i]
                # Calculate World position ID
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i],
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

                # print(tvecs)
                # print("Detected")
                # print(corners[i])
                # print(self.marker_size)
                # print(self.camera_matrix)
                # print(self.dist_coeffs)

                ret = rotate_marker_center(rvecs, self.marker_size, tvecs)

                # Apply the low-pass filter
                filtered_position = self.apply_low_pass_filter(
                    ret,
                    ids[i][0],
                    alpha=0.1,
                )

                # Store the position in the dictionary
                self.__detected_markers_world[ids[i][0]] = filtered_position

        self.draw_ar()

        # cv2.imshow("Image", self.image)
        cv2.waitKey(3)

    def calculate_world_position(self, request):
        
        closest_marker_id = None
        closest_marker_distance = np.inf
        closest_marker_corners = None

        center = request.center.data

        # closest_marker_id = None
        closest_marker_distance = float('inf')
        # closest_marker_corners = None

        # Iterate through detected markers
        for marker_id, corners in self.__detected_markers_corners.items():
            center_marker = [(corners[0][2][0] + corners[0][0][0])/2, (corners[0][2][1] + corners[0][0][1])/2]
            min_distance = np.linalg.norm(np.array(center) - np.array(center_marker))
            
            # Check if the current marker is closer than the previous closest marker
            if min_distance < closest_marker_distance:
                closest_marker_id = marker_id
                closest_marker_distance = min_distance
                closest_marker_corners = corners

        if closest_marker_id is not None:

            # Calculate the corners of the rectangle with the same side lengths, centered at [center_x, center_y]
            half_width = (np.abs(closest_marker_corners[0][0][0] - closest_marker_corners[0][3][0])) / 2

            corners_target = np.array([[[center[0] + half_width, center[1] - half_width],
                [center[0] + half_width, center[1] + half_width],
                [center[0] - half_width, center[1] + half_width],
                [center[0] - half_width, center[1] - half_width]]], dtype=np.float32)

            # Calculate World position ID
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners_target,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs,
            )

            ret = rotate_marker_center(rvecs, self.marker_size, tvecs)

            target_position_world = Point()
            target_position_world.x = np.round(
                ret[0], 2
            )
            target_position_world.y = np.round(
                ret[1], 2
            )
            target_position_world.z = np.round(
                ret[2], 2
            )
            self.__target_camera_pub.publish(target_position_world)

        return True

    def draw_ar(self):

        # If marker is detected
        if self.marker_id in self.__detected_markers_corners:

            # Calculate the center of the marker
            self.center_x = int(
                np.mean(
                    self.__detected_markers_corners[self.marker_id][0][:, 0]
                )
            )
            self.center_y = int(
                np.mean(
                    self.__detected_markers_corners[self.marker_id][0][:, 1]
                )
            )

            self.check_target_size()
            self.check_expiration_date()

            target_position_world = Point()
            target_position_world.x = np.round(
                self.__detected_markers_world[self.marker_id][0], 2
            )
            target_position_world.y = np.round(
                self.__detected_markers_world[self.marker_id][1], 2
            )
            target_position_world.z = np.round(
                self.__detected_markers_world[self.marker_id][2], 2
            )
            self.__target_camera_pub.publish(target_position_world)

        # If marker is not detected
        else:
            if self.marker_id is not None:

                if self.robot_state == 0:

                    # Call service with help request
                    self.remote_help_service(1)

                    self.robot_state = 1

                self.center_x = 0
                self.center_y = 0

                # print("Object not detected!")

        target_position_frame = Float32MultiArray()
        target_position_frame.data = [self.center_x, self.center_y]
        self.__target_position_frame_pub.publish(target_position_frame)

    def check_target_size(self):
        if self.marker_size == 0.05:
            pass
            # Send help

    def check_expiration_date(self):

        # Parse the input string to extract month and year
        try:
            input_date = datetime.strptime(self.expiration, "%m%Y")
        except ValueError:
            print("Invalid input format. Please provide a valid MMYYYY string.")

        # Get the current date
        current_date = datetime.now()

        # Check if the input date is in the past (expired)
        if input_date < current_date:
            # print(
            #     f"The provided date {input_date.strftime('%B %Y')} has expired."
            # )
            if self.robot_state == 0:
                self.remote_help_service(2)
                self.robot_state = 2


def rotate_marker_center(rvec, markersize, tvec=None):
    mhalf = markersize / 2.0

    # Convert rotation vector to rotation matrix, transforming from marker-world to cam-world
    mrv, _ = cv2.Rodrigues(rvec)

    # Calculate the 3D coordinates of the center in cam-world
    center_cam_world = mhalf * mrv[:, 2]

    # If tvec is given, move the center by tvec
    if tvec is not None:
        center_cam_world += tvec.flatten()

    return center_cam_world


if __name__ == "__main__":

    ar = Ar()

    while not rospy.is_shutdown():
        ar.rate.sleep()
