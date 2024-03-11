#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from datetime import datetime
from cv_bridge import (CvBridge, CvBridgeError)
from std_msgs.msg import (Float32MultiArray, Bool, Int32)
from std_srvs.srv import (SetBool)
from sensor_msgs.msg import (Image)
from geometry_msgs.msg import (Point, Pose)
from gopher_ros_clearcore.msg import (Position)
from holo_project.msg import (TargetInfo)
from Scripts.srv import (UpdateState, UpdateChest, ItemPositionFOV, ConvertTargetPosition)


class ObjectTracker:

    def __init__(self):

        # # Private CONSTANTS:
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )
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

        # # Public CONSTANTS:
        self.RATE = rospy.Rate(5)

        # # Private variables:
        self.__image = None
        self.__counter = None
        self.__chest_position = 440.0

        self.__marker_id = None
        self.__previous_marker_id = None
        self.__expiration = None
        self.__object_center = []
        self.__state = 0
        self.__robot_state = 0  # Running
        self.__user = 0  #0 robot, 1 remote, 2 local

        self.__adjusting_chest = False
        self.__is_target_detected = False
        self.__on_startup = True
        self.__new_target_received = False
        self.__rh_help = False

        self.__chest_cam_anchor_tf = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        # Create dictionary to store position of detected markers
        self.__detected_markers_world = {}
        self.__detected_markers_centers = {}

        # # Service provider:
        rospy.Service(
            '/resume_task',
            UpdateState,
            self.__resume_task,
        )

        rospy.Service(
            '/update_chest',
            UpdateChest,
            self.__update_chest,
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
            '/move_chest',
            UpdateState,
            self.__move_chest_remote,
        )
    
        # # Service subscriber:
        self.__remote_help_service = rospy.ServiceProxy(
            '/remote_help_request_service',
            UpdateState,
        )

        self.__update_target_service = rospy.ServiceProxy(
            '/update_target',
            SetBool,
        )

        self.__remote_handling = rospy.ServiceProxy(
            '/remote_handling',
            SetBool,
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

        self.__chest_position = rospy.Publisher(
            'z_chest_pos',
            Position,
            queue_size=1,
        )

        self.__user_control = rospy.Publisher(
            '/user_control',
            Int32,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/target_identifier',
            TargetInfo,
            self.__target_identifier_callback,
        ),

        rospy.Subscriber(
            f'/my_gen3/pick_and_place',
            Int32,
            self.__robot_pick_and_place_callback,
        )

        rospy.Subscriber(
            '/target_counter',
            Int32,
            self.__counter_callback,
        )

        rospy.Subscriber(
            '/chest_cam/camera/color/image_raw',
            Image,
            self.__image_callback,
        ),

        rospy.Subscriber(
            '/my_gen3/tf_chest_cam_anchor',
            Pose,
            self.__tf_chest_cam_anchor_callback,
        ),

        rospy.Subscriber(
            '/rh_help',
            Bool,
            self.__remote_help_callback,
        ),

    def __image_callback(self, data):

        try:

            self.__image = self.__BRIDGE.imgmsg_to_cv2(data, "bgr8")

            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

        if not self.__adjusting_chest:

            # Detect ArUco markers
            corners, ids, _ = self.__ARUCO_DETECTOR.detectMarkers(self.__image)

            if ids is not None:
                for i in range(len(ids)):

                    # Calculate World position ID
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i],
                        self.__MARKER_SIZE,
                        self.__CAMERA_MATRIX,
                        self.__DIST_COEFFS,
                    )

                    ret = rotate_marker_center(rvecs, self.__MARKER_SIZE, tvecs)

                    position_target = Float32MultiArray()
                    position_target.data = [ret[0], ret[1], ret[2] + 0.05]

                    target = self.__convert_target_service(position_target)

                    # Apply the low-pass filter
                    filtered_position = self.__apply_low_pass_filter(
                        np.array(target.fromanchor.data),
                        ids[i][0],
                        alpha=0.1,
                    )

                    # Store the position in the dictionary
                    self.__detected_markers_world[ids[i][0]] = filtered_position

        self.__draw_ar()
    
    def __robot_pick_and_place_callback(self, message):

        self.__state = message.data

    def __remote_help_callback(self, message):

        self.__rh_help = message.data

    def __tf_chest_cam_anchor_callback(self, message):
        self.__chest_cam_anchor_tf['position'][0] = message.position.x
        self.__chest_cam_anchor_tf['position'][1] = message.position.y
        self.__chest_cam_anchor_tf['position'][2] = message.position.z

    def __counter_callback(self, message):

        self.__counter = message.data

    def __target_identifier_callback(self, message):

        self.__marker_id = message.id
        self.__expiration = message.expiration

        if self.__marker_id != self.__previous_marker_id:

            self.__new_target_received = True
            self.__previous_marker_id = self.__marker_id

    def __update_chest(self, request):

        detected_marker_y = self.__detected_markers_world[self.__marker_id][1]
        chest_pos_y = self.__chest_cam_anchor_tf['position'][1]
        diff = detected_marker_y - chest_pos_y

        if diff > 0.20 and self.__chest_position == 200:
            new_position = 440
            self.__move_chest(new_position)
            return int(new_position)
        elif diff < -0.20 and self.__chest_position == 440:
            new_position = 200
            self.__move_chest(new_position)
            return int(new_position)

        elif self.__state == 3 and self.__chest_position == 440:
            self.__move_chest(200.0)
            return 200

        elif detected_marker_y - chest_pos_y < -0.10 and self.__chest_position == 440:

            self.__move_chest(200.0)
            return 200

        return int(self.__chest_position)

    def __resume_task(self, request):

        self.__robot_state = 0

        if self.__marker_id in self.__detected_markers_world and request.state == 0:
            self.__remote_handling(True)
            self.__change_task_state_service(0)
        else:
            self.__update_target_service(True)
            self.__remote_help_service(0)

        self.__user = 0

        return True

    def __local_help(self, request):

        self.__local_help_service(request.state)

        self.__user = 2
        return True
    
    def __move_chest_remote(self, request):

        if request.state == 0 and self.__chest_position == 200:
            self.__move_chest(440.0)
        elif request.state == 1 and self.__chest_position == 440:
            self.__move_chest(200.0)

        return True
    
    def __calculate_world_position(self, request):

        closest_marker_id = None
        closest_marker_distance = np.inf
        closest_marker_corners = None

        center = request.center.data

        # closest_marker_id = None
        closest_marker_distance = float('inf')
        # closest_marker_corners = None

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

                ret = rotate_marker_center(rvecs, self.__MARKER_SIZE, tvecs)

                position_target = Float32MultiArray()
                position_target.data = ret

                target = self.__convert_target_service(position_target)
                target = np.array(target.fromanchor.data)

                self.__detected_markers_world[self.__marker_id] = target

                self.__detected_markers_centers[self.__marker_id
                                               ] = [center[0], center[1]]

        return True

    def __draw_ar(self):

        target_position_world = Point()

        # If marker is detected
        if self.__marker_id in self.__detected_markers_world:

            corners, ids, _ = self.__ARUCO_DETECTOR.detectMarkers(self.__image)

            if ids is not None and self.__marker_id in ids and not self.__is_expired():

                for i in range(len(ids)):

                    if ids[i] == self.__marker_id:

                        self.__object_center = [int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))]

                        self.__detected_markers_centers[self.__marker_id] = self.__object_center

                self.__is_target_detected = True
            else:

                if self.__marker_id not in self.__detected_markers_centers:
                    self.__detected_markers_centers[self.__marker_id] = [0, 0]

                    self.__object_center = [0, 0]

                    self.__is_target_detected = False

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

            if self.__robot_state != 0:

                target_position_world.x = 0
                target_position_world.y = 0
                target_position_world.z = 0

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
        
    def main_loop(self):

        if self.__new_target_received:

            if self.__marker_id not in self.__detected_markers_world:

                if self.__marker_id is not None:

                    if self.__robot_state == 0:

                        # Call service with help request
                        self.__remote_help_service(1)
                        self.__change_task_state_service(1)

                        self.__robot_state = 1

            else:
                # Check expiration date
                if self.__is_expired():

                    if self.__robot_state == 0:
                        self.__remote_help_service(2)
                        self.__change_task_state_service(2)
                        self.__robot_state = 2

                else:
                    self.__change_task_state_service(0)

                    self.__user = 0

            self.__new_target_received = False

        if self.__rh_help and self.__user != 2:
            self.__user = 1

        user_in_charge = Int32()
        user_in_charge.data = self.__user
        self.__user_control.publish(user_in_charge)

    def __move_chest(self, desired_position):

        self.__adjusting_chest = True

        chest_pos = Position()
        chest_pos.position = desired_position
        chest_pos.velocity = 1.0
        self.__chest_position.publish(chest_pos)

        self.__chest_position = desired_position

        rospy.sleep(5)

        self.__adjusting_chest = False

    # # Public methods:
    def adjust_chest(self):

        if self.__counter == -1 and self.__on_startup:

            self.__move_chest(200.0)
            self.__on_startup = False

            self.__update_target_service(True)
        
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

def main():
    """
    """

    rospy.init_node(
        'object_tracker',
        log_level=rospy.INFO,
    )

    rospy.loginfo('\n\n\n\n\n')

    object_tracker = ObjectTracker()

    while not rospy.is_shutdown():
        object_tracker.main_loop()
        object_tracker.adjust_chest()
        object_tracker.RATE.sleep()

if __name__ == "__main__":
    main()