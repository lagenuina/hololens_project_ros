#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from datetime import datetime
import time
from cv_bridge import (CvBridge, CvBridgeError)
from std_msgs.msg import (Float32MultiArray, Bool, Int32)
from std_srvs.srv import (SetBool, Empty)
from sensor_msgs.msg import (Image, CameraInfo)
from geometry_msgs.msg import (Point)
from holo_project.msg import (TargetInfo)
from Scripts.srv import (
    UpdateState,
    ItemPositionFOV,
    ConvertTargetPosition,
    SendFloat32MultiArray,
    ReceiveInt,
)


class ObjectTracker:

    def __init__(
        self,
        node_name,
        anchor_id,
        task,
    ):

        # # Private CONSTANTS:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        self.__ARUCO_DETECTOR = cv2.aruco.ArucoDetector(
            aruco_dict,
            aruco_params,
        )
        self.__CAMERA_MATRIX = np.array(
            [
                [605.3272705078125, 0.0, 312.21490478515625],
                [0.0, 605.3390502929688, 253.79823303222656],
                [0.0, 0.0, 1.0],
            ]
        )
        self.__DIST_COEFFS = np.array([0, 0, 0, 0, 0])
        self.__MARKER_SIZE = 0.03
        self.__BRIDGE = CvBridge()
        self.__NODE_NAME = node_name

        # # Public CONSTANTS:
        self.RATE = rospy.Rate(10)

        # # Private variables:
        self.__image = None
        self.__depth_image = None

        self.__marker_id = None
        self.__previous_marker_id = None
        self.__failure_sent = False
        self.__expiration = None
        self.__object_center = []
        self.__robot_state = 0  # Running
        self.__boxes = [100, 101, 102]

        self.__error_code = 0
        self.__assign_to = 0
        self.__is_tracking = False
        self.__new_target_received = False
        self.__restoking = False
        self.__is_anchor_set = False

        self.__counter_medicines = 0

        self.ANCHOR_ID = anchor_id

        if task:
            self.__restoking_id = 28
        else:
            self.__restoking_id = 100

        # Create dictionary to store position of detected markers
        self.__detected_markers_world = {}
        self.__detected_markers_centers = {}

        # # Service provider:
        rospy.Service(
            f'{self.__NODE_NAME}/pause',
            SetBool,
            self.__pause_object_tracking,
        )
        rospy.Service(
            '/resume_task',
            UpdateState,
            self.__resume_task,
        )
        rospy.Service(
            '/resume_task_local',
            Empty,
            self.__resume_task_local,
        )
        rospy.Service(
            '/calculate_world_position_service',
            ItemPositionFOV,
            self.__calculate_world_position,
        )

        rospy.Service(
            '/local_request',
            UpdateState,
            self.__local_help,
        )

        rospy.Service(
            '/grasp_failure',
            Empty,
            self.__grasp_failure,
        )

        # # Service subscriber:
        self.__update_target_service = rospy.ServiceProxy(
            '/update_target',
            Empty,
        )

        self.__record_failure = rospy.ServiceProxy(
            '/record_failure',
            Empty,
        )

        self.__remote_handling = rospy.ServiceProxy(
            '/remote_handling',
            SetBool,
        )

        self.__remote_help_service = rospy.ServiceProxy(
            '/remote_help_request_service',
            UpdateState,
        )

        self.__change_task_state_service = rospy.ServiceProxy(
            '/change_task_state_service',
            UpdateState,
        )

        self.__convert_target_service = rospy.ServiceProxy(
            '/from_chest_to_anchor',
            ConvertTargetPosition,
        )

        self.__local_help_service = rospy.ServiceProxy(
            '/local_help_request_service',
            UpdateState,
        )

        self.__move_chest = rospy.ServiceProxy(
            '/move_chest',
            UpdateState,
        )

        self.__set_anchor_service = rospy.ServiceProxy(
            '/set_anchor',
            SendFloat32MultiArray,
        )

        self.__assign_failure_service = rospy.ServiceProxy(
            '/assign_failure',
            ReceiveInt,
        )

        self.__failure_resolved_service = rospy.ServiceProxy(
            '/failure_resolved',
            Empty,
        )

        # # Topic publisher:
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

        self.__error_pub = rospy.Publisher(
            '/object_tracker/error_code',
            Int32,
            queue_size=1,
        )

        self.__restocking_pub = rospy.Publisher(
            '/object_tracker/restocking',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/task_manager/target_identifier',
            TargetInfo,
            self.__target_identifier_callback,
        ),

        rospy.Subscriber(
            '/chest_cam/camera/color/image_raw',
            Image,
            self.__image_callback,
        ),

        rospy.Subscriber(
            '/chest_cam/camera/aligned_depth_to_color/image_raw',
            Image,
            self.__depth_image_callback,
        )

    def __depth_image_callback(self, data):

        try:
            # Convert depth image to a CV2 image (16UC1 encoding typically used for depth images)
            self.__depth_image = self.__BRIDGE.imgmsg_to_cv2(
                data, desired_encoding="16UC1"
            )

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def __image_callback(self, data):

        try:

            self.__image = self.__BRIDGE.imgmsg_to_cv2(data, "bgr8")

            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

        if self.__is_tracking and not self.__restoking:
            self.__detect_and_store()

    def __target_identifier_callback(self, message):

        self.__marker_id = message.id
        self.__expiration = message.expiration

        if self.__marker_id != self.__previous_marker_id:

            rospy.sleep(3)

            if (
                self.__marker_id == self.__restoking_id
            ) and self.__counter_medicines > 4:

                self.__move_chest(0)
                self.__restoking = True

                # Empty all previously collected target poses
                self.__detected_markers_world.clear()
                self.__detected_markers_centers.clear()

            self.__new_target_received = True
            self.__previous_marker_id = self.__marker_id

            self.__failure_sent = False
            self.__counter_medicines += 1

    def __resume_task_local(self, request):

        self.__robot_state = 0
        self.__failure_resolved_service()

        self.__assign_to = 0

        return []

    def __resume_task(self, request):

        self.__robot_state = 0

        if self.__marker_id in self.__detected_markers_world and request.state == 0:
            self.__remote_handling(True)
            self.__change_task_state_service(0)
        else:
            self.__update_target_service()
            self.__remote_help_service(0)

        return True

    def __grasp_failure(self, request):

        if not self.__failure_sent:

            operator_number = self.__assign_failure_service(4)
            self.__assign_to = operator_number.response

            print("Different id!", self.__assign_to)
            self.__failure_sent = True

        self.__error_code = 4

        if self.__assign_to == 1:
            self.__remote_help_service(self.__error_code)
        elif self.__assign_to == 2:
            self.__local_help_service(self.__error_code)

        rospy.loginfo(
            f'\033[92mFailure 4 assigned to operator {self.__assign_to}.\033[0m',
        )

        return []

    def __pause_object_tracking(self, request):

        self.__is_tracking = not (request.data)

        if self.__is_tracking:
            message = "Object Tracking was resumed."
        else:
            message = "Object Tracking was paused."

        return [True, message]

    def __local_help(self, request):

        self.__local_help_service(request.state)

        return True

    def __calculate_world_position(self, request):

        closest_marker_id = None
        closest_marker_distance = np.inf
        closest_marker_corners = None

        center = request.center.data

        closest_marker_distance = float('inf')

        corners, ids, _ = self.__ARUCO_DETECTOR.detectMarkers(self.__image)

        detected_markers_corners = {}

        if ids is not None:
            for i in range(len(ids)):

                detected_markers_corners[ids[i][0]] = corners[i]

        else:
            return False

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

        if closest_marker_id is not None:

            if closest_marker_distance < 50 and (
                self.__marker_id != closest_marker_id
            ):

                self.__detected_markers_world[
                    self.__marker_id
                ] = self.__detected_markers_world[closest_marker_id]

                self.__detected_markers_centers[self.__marker_id] = [
                    int(np.mean(closest_marker_corners[0][:, 0])),
                    int(np.mean(closest_marker_corners[0][:, 1]))
                ]

            else:
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
                    self.__MARKER_SIZE,
                    self.__CAMERA_MATRIX,
                    self.__DIST_COEFFS,
                )

                marker_center = np.mean(
                    corners_target[0], axis=0
                )  # Average corners to get center

                if self.__depth_image is not None:
                    # Get the depth value at the mapped coordinates
                    depth_value = self.__depth_image[int(marker_center[1]),
                                                     int(marker_center[0])]

                    # Convert depth value from millimeters to meters (if needed)
                    depth_value_meters = depth_value / 1000.0

                    if depth_value_meters > 0:

                        # Set the depth value in the translation vector
                        tvecs[0][0][2] = depth_value_meters

                position_target = Float32MultiArray()
                position_target.data = position_target.data = [
                    tvecs[0][0][0], tvecs[0][0][1], depth_value_meters + 0.12
                ]

                target = self.__convert_target_service(position_target)
                target = np.array(target.fromanchor.data)

                self.__detected_markers_world[self.__marker_id] = target

                self.__detected_markers_centers[self.__marker_id
                                               ] = [center[0], center[1]]

        return True

    def __detect_and_store(self):

        # Detect ArUco markers
        corners, ids, _ = self.__ARUCO_DETECTOR.detectMarkers(self.__image)

        if ids is not None:
            for i in range(len(ids)):

                marker_center = np.mean(
                    corners[i][0], axis=0
                )  # Average corners to get center

                # Estimate pose of the marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i],
                    self.__MARKER_SIZE,
                    self.__CAMERA_MATRIX,
                    self.__DIST_COEFFS,
                )

                if self.__depth_image is not None:
                    # Get the depth value at the mapped coordinates
                    depth_value = self.__depth_image[int(marker_center[1]),
                                                     int(marker_center[0])]

                    # Convert depth value from millimeters to meters (if needed)
                    depth_value_meters = depth_value / 1000.0

                position_target = Float32MultiArray()
                position_target.data = [
                    tvecs[0][0][0], tvecs[0][0][1], depth_value_meters + 0.12
                ]

                if not self.__is_anchor_set:

                    if ids[i] == self.ANCHOR_ID:
                        self.__set_anchor_service(position_target)
                        self.__is_anchor_set = True

                        rospy.sleep(5)

                        print("")
                        rospy.loginfo(f'\033[92mReady to start!\033[0m',)

                else:

                    target = self.__convert_target_service(position_target)

                    # Apply the low-pass filter
                    filtered_position = self.__apply_low_pass_filter(
                        np.array(target.fromanchor.data),
                        ids[i][0],
                        alpha=0.1,
                    )

                    # Store the position in the dictionary
                    self.__detected_markers_world[ids[i][0]] = filtered_position

    def __publish_target_pose(self):

        target_position_world = Point()

        # If marker is detected
        if self.__marker_id in self.__detected_markers_world:

            corners, ids, _ = self.__ARUCO_DETECTOR.detectMarkers(self.__image)

            if ids is not None and self.__marker_id in ids and not self.__is_expired(
            ):

                for i in range(len(ids)):

                    if ids[i] == self.__marker_id:

                        self.__object_center = [
                            int(np.mean(corners[i][0][:, 0])),
                            int(np.mean(corners[i][0][:, 1]))
                        ]

                        self.__detected_markers_centers[self.__marker_id
                                                       ] = self.__object_center

            else:

                if self.__marker_id not in self.__detected_markers_centers:
                    self.__detected_markers_centers[self.__marker_id] = [0, 0]

                    self.__object_center = [0, 0]

            if self.__marker_id in self.__detected_markers_world:
                target_position_world.x = np.round(
                    self.__detected_markers_world[self.__marker_id][0], 2
                )
                target_position_world.y = np.round(
                    self.__detected_markers_world[self.__marker_id][1], 2
                )
                target_position_world.z = np.round(
                    self.__detected_markers_world[self.__marker_id][2], 2
                )

                center_in_frame = Float32MultiArray()
                center_in_frame.data = self.__detected_markers_centers[
                    self.__marker_id]
                self.__target_position_frame_pub.publish(center_in_frame)

        else:

            target_position_world.x = 100
            target_position_world.y = 100
            target_position_world.z = 100

        self.__target_camera_pub.publish(target_position_world)

    def __is_expired(self):

        # Parse the input string to extract month and year
        try:
            input_date = datetime.strptime(self.__expiration, "%m%Y")
        except ValueError:
            print("Invalid input format. Please provide a valid MMYYYY string.")

        # Get the current date
        current_date = datetime.now()

        # Check if the input date is in the past (expired)
        if input_date < current_date:
            return True
        else:
            return False

    def __apply_low_pass_filter(self, current_position, marker_id, alpha):

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

    def __send_help_request(self, assign_to, error_code):

        if assign_to == 1:
            self.__remote_help_service(error_code)
        elif assign_to == 2:
            self.__local_help_service(error_code)

        if error_code != 3:
            self.__change_task_state_service(error_code)
            self.__robot_state = error_code

    def __start_restocking(self):

        if self.__restoking:
            # Start the timer when restocking is initiated
            if not hasattr(self, 'restocking_start_time'):
                rospy.loginfo(f'\033[92mStart restocking!\033[0m',)
                self.restocking_start_time = time.time()
                self.last_print_time = self.restocking_start_time  # To track last time a message was printed

            # Calculate elapsed time since restocking started
            elapsed_time = time.time() - self.restocking_start_time
            remaining_time = 60 - elapsed_time  # Calculate remaining time

            # Check if 60 seconds have passed since restocking started
            if elapsed_time >= 60:
                self.__restoking = False

                rospy.loginfo(f'\033[92mRestocking finished.\033[0m',)
                del self.restocking_start_time  # Clear the timer
                del self.last_print_time  # Clear the last print time
            else:
                # Print the remaining time every 20 seconds
                if time.time() - self.last_print_time >= 20:
                    rospy.loginfo(
                        f'\033[93mTime left for restocking: {int(remaining_time + 1)} seconds\033[0m',
                    )
                    self.last_print_time = time.time()  # Update last print time

    def main_loop(self):

        if self.__restoking:
            self.__start_restocking()

        if self.__new_target_received and not self.__restoking:

            self.__error_code = None

            if self.__marker_id is not None and self.__robot_state == 0:

                if self.__marker_id in self.__boxes:

                    # Failure 3 - Medicine is ungraspable
                    self.__error_code = 3

                elif self.__marker_id not in self.__detected_markers_world:

                    # Failure 1 - Marker not detected/misplaced
                    self.__error_code = 1

                else:
                    if self.__is_expired():

                        # Failure 2 - Medicine is expired
                        self.__error_code = 2

                    else:
                        # No failure
                        self.__change_task_state_service(0)
                        self.__error_code = 0

                        self.__assign_to = 0

            if self.__error_code is not None and self.__error_code != 0:

                operator_number = self.__assign_failure_service(
                    self.__error_code
                )
                self.__assign_to = operator_number.response

                self.__send_help_request(
                    self.__assign_to,
                    self.__error_code,
                )

                rospy.loginfo(
                    f'\033[92mFailure {self.__error_code} assigned to operator {self.__assign_to}.\033[0m',
                )

                if self.__error_code == 3:
                    if self.__assign_to == 1:
                        rospy.sleep(5)
                        self.__record_failure()
                        self.__assign_to = 0

            self.__new_target_received = False

        self.__publish_target_pose()

        if self.__error_code is not None:
            error = Int32()
            error.data = self.__error_code
            self.__error_pub.publish(error)

        restockin_bool = Bool()
        restockin_bool.data = self.__restoking
        self.__restocking_pub.publish(restockin_bool)


def main():
    """
    """

    rospy.init_node(
        'object_tracker',
        log_level=rospy.INFO,
    )

    rospy.loginfo('\n\n\n\n\n')

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    id = rospy.get_param(param_name=f'{rospy.get_name()}/anchor_id',)

    task = rospy.get_param(
        param_name=f'{rospy.get_name()}/study',
        default='false',
    )

    object_tracker = ObjectTracker(
        node_name=node_name,
        anchor_id=id,
        task=task,
    )

    while not rospy.is_shutdown():
        object_tracker.main_loop()
        object_tracker.RATE.sleep()


if __name__ == "__main__":
    main()
