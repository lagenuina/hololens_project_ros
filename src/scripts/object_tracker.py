#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Bool, Int32
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import (Point)

from gopher_ros_clearcore.msg import (Position)
import tf
from datetime import datetime
from holo_project.msg import TargetInfo
from Scripts.srv import UpdateState, UpdateChest, ItemPositionFOV, ConvertTargetPosition, BoolUpdate


class Ar:

    def __init__(self):

        rospy.init_node("interface")

        self.bridge = CvBridge()
        self.rate = rospy.Rate(5)

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

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

        self.target_detected = False
        self.on_startup = True
        self.counter = None
        self.adjusting_chest = False
        self.marker_size = 0.02
        self.in_box = False
        self.previous_marker_id = None
        self.marker_id = None
        self.expiration = None
        self.center_x = 0
        self.center_y = 0

        self.chest_position = 440.0
        self.robot_state = 0  # Running
        self.new_target_received = False

        # Create dictionary to store position of detected markers
        self.__detected_markers_world = {}
        self.__detected_markers_corners = {}

        self.box_ids = [20, 21, 22]

        # Camera calibration parameters (replace with your actual values)
        # self.camera_matrix = np.array(
        #     [
        #         [909.1905517578125, 0.0, 644.7723999023438],
        #         [0.0, 908.4768676757812, 377.5361633300781],
        #         [0.0, 0.0, 1.0],
        #     ]
        # )

        self.camera_matrix = np.array(
            [
                [605.3272705078125, 0.0, 312.21490478515625],
                [0.0, 605.3390502929688, 253.79823303222656],
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
            '/target_counter',
            Int32,
            self.counter_callback,
        )

        rospy.Subscriber(
            '/chest_cam/camera/color/image_raw',
            Image,
            self.image_callback,
        ),

        # Publishers
        self.__target_camera_pub = rospy.Publisher(
            '/my_gen3/target_workspace_cam',
            Point,
            queue_size=1,
        )

        # self.__target_position_frame_pub = rospy.Publisher(
        #     '/workspace_cam/target_position_in_frame',
        #     Float32MultiArray,
        #     queue_size=1,
        # )

        self.__chest_position = rospy.Publisher(
            'z_chest_pos',
            Position,
            queue_size=1,
        )

        self.image_pub = rospy.Publisher(
            "/chest_cam/remote_interface",
            Image,
            queue_size=1,
        )

        # Service provider
        self.resume_task_service = rospy.Service(
            '/resume_task',
            UpdateState,
            self.resume_task,
        )

        self.update_chest_service = rospy.Service(
            '/update_chest',
            UpdateChest,
            self.update_chest,
        )

        # Service provider
        self.position_fov_service = rospy.Service(
            '/calculate_world_position_service',
            ItemPositionFOV,
            self.calculate_world_position,
        )

        self.remote_service = rospy.Service(
            '/local_request',
            UpdateState,
            self.local_help,
        )

        # Service subscriber
        self.remote_help_service = rospy.ServiceProxy(
            '/remote_help_request_service',
            UpdateState,
        )

        # self.pause_task_service = rospy.ServiceProxy(
        #     '/pause_task_service',
        #     UpdateState,
        # )

        self.update_target_service = rospy.ServiceProxy(
            '/update_target',
            BoolUpdate,
        )

        self.change_task_state_service = rospy.ServiceProxy(
            '/change_task_state_service',
            UpdateState,
        )

        self.convert_target_service = rospy.ServiceProxy(
            '/from_chest_to_anchor',
            ConvertTargetPosition,
        )

        self.local_help_service = rospy.ServiceProxy(
            '/local_help_request_service',
            UpdateState,
        )

    def local_help(self, request):

        self.rh_help = False
        self.local_help_service(request)
        return True

    def update_chest(self, request):

        if self.__detected_markers_world[self.marker_id][1] > 0.25:
            if self.chest_position == 200:
                self.move_chest(440.0)
                return int(440)
            elif self.chest_position == 440:
                self.move_chest(200.0)
                return int(200)

        elif self.__detected_markers_world[
            self.marker_id][1] < 0.25 and self.chest_position == 440:
            self.move_chest(200.0)
            return int(200)

        return int(self.chest_position)

    def resume_task(self, request):

        print("Task resumed.")
        self.robot_state = 0

        if self.in_box:
            self.update_target_service(True)
        elif self.marker_id in self.__detected_markers_world:
            self.change_task_state_service(0)
        else:
            self.update_target_service(True)

        self.remote_help_service(0)

        return True

    def counter_callback(self, message):

        self.counter = message.data

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
            self.in_box = True
        else:
            self.in_box = False

        if self.marker_id != self.previous_marker_id:

            self.new_target_received = True
            self.previous_marker_id = self.marker_id

    def image_callback(self, data):

        try:

            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

        if not self.adjusting_chest:

            # Detect ArUco markers
            corners, ids, _ = self.aruco_detector.detectMarkers(self.image)

            if ids is not None:
                for i in range(len(ids)):

                    # self.__detected_markers_corners[ids[i][0]] = corners[i]

                    # Calculate World position ID
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i],
                        self.marker_size,
                        self.camera_matrix,
                        self.dist_coeffs,
                    )

                    # print(ids[i], marker_size)
                    ret = rotate_marker_center(rvecs, self.marker_size, tvecs)

                    position_target = Float32MultiArray()
                    position_target.data = [ret[0], ret[1], ret[2] + 0.05]

                    target = self.convert_target_service(position_target)

                    # Apply the low-pass filter
                    filtered_position = self.apply_low_pass_filter(
                        np.array(target.fromanchor.data),
                        ids[i][0],
                        alpha=0.1,
                    )

                    # Store the position in the dictionary
                    self.__detected_markers_world[ids[i][0]] = filtered_position

        self.draw_ar()

        # cv2.waitKey(3)

    def calculate_world_position(self, request):

        closest_marker_id = None
        closest_marker_distance = np.inf
        closest_marker_corners = None

        center = request.center.data

        # closest_marker_id = None
        closest_marker_distance = float('inf')
        # closest_marker_corners = None

        corners, ids, _ = self.aruco_detector.detectMarkers(self.image)

        detected_markers_corners = {}

        if ids is not None:
            for i in range(len(ids)):

                detected_markers_corners[ids[i][0]] = corners[i]

        # Iterate through detected markers
        for marker_id, corners in detected_markers_corners.items():
            center_marker = [
                (corners[0][2][0] + corners[0][0][0]) / 2,
                (corners[0][2][1] + corners[0][0][1]) / 2
            ]
            min_distance = np.linalg.norm(
                np.array(center) - np.array(center_marker)
            )

            # Check if the current marker is closer than the previous closest marker
            if min_distance < closest_marker_distance:
                closest_marker_id = marker_id
                closest_marker_distance = min_distance
                closest_marker_corners = corners

        # print(closest_marker_id, closest_marker_distance)

        if closest_marker_id is not None:
            # Calculate the corners of the rectangle with the same side lengths, centered at [center_x, center_y]
            half_width = (
                np.abs(
                    closest_marker_corners[0][0][0]
                    - closest_marker_corners[0][2][0]
                )
            ) / 2

            corners_target = np.array(
                [
                    [
                        [center[0] - half_width, center[1] - half_width],
                        [center[0] + half_width, center[1] - half_width],
                        [center[0] + half_width, center[1] + half_width],
                        [center[0] - half_width, center[1] + half_width],
                    ]
                ],
                dtype=np.float32
            )

            # Calculate World position ID
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners_target,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs,
            )

            ret = rotate_marker_center(rvecs, self.marker_size, tvecs)

            position_target = Float32MultiArray()
            position_target.data = ret

            target = self.convert_target_service(position_target)
            target = np.array(target.fromanchor.data)

            self.__detected_markers_world[self.marker_id] = target
            # self.__detected_markers_corners[self.marker_id] = corners_target

            # target_position_world = Point()
            # target_position_world.x = np.round(target[0], 2)
            # target_position_world.y = np.round(target[1], 2)
            # target_position_world.z = np.round(target[2], 2)
            # self.__target_camera_pub.publish(target_position_world)

            # print(target_position_world)
        return True

    def draw_ar(self):

        # print(self.__detected_markers_world.keys())
        print(self.__detected_markers_world)
        target_position_world = Point()

        # If marker is detected
        if self.marker_id in self.__detected_markers_world:

            corners, ids, _ = self.aruco_detector.detectMarkers(self.image)

            if ids is not None and self.marker_id in ids:

                for i in range(len(ids)):

                    if ids[i] == self.marker_id:

                        self.center_x = int(np.mean(corners[i][0][:, 0]))
                        self.center_y = int(np.mean(corners[i][0][:, 1]))

                self.target_detected = True
            else:
                self.center_x = 0
                self.center_y = 0

                self.target_detected = False

            target_position_world.x = np.round(
                self.__detected_markers_world[self.marker_id][0], 2
            )
            target_position_world.y = np.round(
                self.__detected_markers_world[self.marker_id][1], 2
            )
            target_position_world.z = np.round(
                self.__detected_markers_world[self.marker_id][2], 2
            )

        else:

            if self.robot_state != 0:
                # self.center_x = 0
                # self.center_y = 0
                target_position_world.x = 0
                target_position_world.y = 0
                target_position_world.z = 0

        self.__target_camera_pub.publish(target_position_world)

    # def check_target_size(self):

    #     if self.in_box:

    #         # self.__detected_markers_world[self.marker_id][2] -= 0.15
    #         # Send help
    #         self.remote_help_service(3)
    #         self.change_task_state_service(3)
    #         self.robot_state = 3

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
            return True
        else:
            return False

    def main_loop(self):

        if self.new_target_received:

            if self.marker_id not in self.__detected_markers_world:

                if self.marker_id is not None:
                    if self.robot_state == 0:

                        # Call service with help request
                        self.remote_help_service(1)
                        self.change_task_state_service(1)

                        self.robot_state = 1

            else:

                # Check target size
                if self.in_box:
                    # Send help
                    self.remote_help_service(3)
                    self.change_task_state_service(3)
                    self.robot_state = 3

                # Check expiration date
                elif self.check_expiration_date():

                    if self.robot_state == 0:
                        self.remote_help_service(2)
                        self.change_task_state_service(2)
                        self.robot_state = 2

                else:
                    self.change_task_state_service(0)

            self.new_target_received = False

        # if (self.center_x == 0) and (self.center_y == 0):
        #     self.target_detected = False
        # else:
        #     self.target_detected = True

        if self.image is not None:
            self.image = self.image.copy()

            if self.target_detected:
                cv2.circle(
                    self.image, (self.center_x, self.center_y),
                    radius=30,
                    color=(0, 255, 0),
                    thickness=2
                )

            # image = CompressedImage()
            # # image.header = rospy.Time.now()
            # image.format = "jpeg"
            # image.data = np.array(cv2.imencode('.jpg', self.image)[1]).tobytes()

            # self.image_pub.publish(image)
            image = Image()
            image.data = np.array(cv2.imencode('.png', self.image)[1]).tobytes()

            self.image_pub.publish(image)

    def adjust_chest(self):

        if self.counter == -1 and self.on_startup:

            self.move_chest(200.0)
            self.on_startup = False

            self.update_target_service(True)

    def move_chest(self, desired_position):

        self.adjusting_chest = True

        chest_pos = Position()
        chest_pos.position = desired_position
        chest_pos.velocity = 1.0
        self.__chest_position.publish(chest_pos)

        self.chest_position = desired_position

        rospy.sleep(5)

        self.adjusting_chest = False


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
        ar.main_loop()
        ar.adjust_chest()
        ar.rate.sleep()
