#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Image, CompressedImage
import cv2
import csv
from datetime import datetime
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from Scripts.msg import TargetInfo
from Scripts.srv import UpdateState, ItemPositionFOV


class RhInterface:

    def __init__(self):

        rospy.init_node("rh_interface")

        self.bridge = CvBridge()
        self.rate = rospy.Rate(100)

        self.image = None
        self.width = None
        self.height = None
        self.center_x = None
        self.center_y = None
        self.target_detected = False
        self.state = 0

        self.box = [20, 21, 22]
        self.last_seen_markers = {}
        self.file_path = '/home/fetch/catkin_workspaces/hololens_ws/src/holo_project/src/scripts/id_medicine.csv'
        self.medicine_info = self.read_file()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )

        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.aruco_params,
        )

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

        rospy.Subscriber(
            '/my_gen3/pick_and_place',
            Int32,
            self.robot_state,
        )

        self.image_pub = rospy.Publisher(
            "/chest_cam/remote_interface",
            Image,
            queue_size=1,
        )

    def image_callback(self, data):

        try:
            self.image = self.bridge.imgmsg_to_cv2(
                data,
                desired_encoding="bgr8",
            )
            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

    def robot_state(self, message):

        self.state = message.data

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

            if self.state in [1, 2, 3, 4]:

                if self.state == 1:
                    text = "Grasping"
                    if self.target_detected:
                        cv2.circle(
                            self.image, (self.center_x, self.center_y),
                            radius=30,
                            color=(0, 255, 0),
                            thickness=2
                        )
                elif self.state == 2:
                    text = "Grasped"
                elif self.state == 3:
                    text = "Placing"
                elif self.state == 4:
                    text = "Placed"

                self.put_text(text)

            if self.state == 0:

                corners, ids, _ = self.aruco_detector.detectMarkers(self.image)
                current_time = rospy.Time.now()

                detected_ids = set()

                # Process detected markers
                if ids is not None:
                    for i, corner in enumerate(corners):
                        id = ids[i][0]
                        detected_ids.add(id)
                        if id in self.medicine_info:
                            medicine_info = self.medicine_info[id]
                            name = medicine_info["name"]
                            expiration = medicine_info["expiration"]

                            # Update last seen position and time
                            self.last_seen_markers[id] = {
                                'corner': corner[0],
                                'time': current_time,
                                'name': name,
                                'expiration': expiration
                            }

                            if id not in self.box:
                                # Display information for detected marker
                                self.display_medicine_info(
                                    corner[0], name, id, expiration
                                )

                # Check for markers that were not detected in this frame
                for id, info in self.last_seen_markers.items():
                    if id not in detected_ids and (current_time
                                                   - info['time']).to_sec() < 3:
                        # Marker was not detected but should still be displayed
                        self.display_medicine_info(
                            info['corner'],
                            info['name'],
                            id,
                            info['expiration'],
                        )

            image = Image()
            # image.data = np.array(cv2.imencode('.png', self.image)[1]).tobytes()
            image = self.bridge.cv2_to_imgmsg(
                self.image,
                encoding="bgr8",
            )

            self.image_pub.publish(image)

    def display_medicine_info(self, corner, name, id, expiration):

        # Calculate the position for the text
        text_position = (int(corner[0][0] - 30), int(corner[0][1] - 80))

        if self.check_expiration_date(expiration):
            color = (0, 40, 255)
        else:
            color = (255, 150, 0)

        # Put the name of the medicine
        cv2.putText(
            self.image, name, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            color, 1
        )

        # Adjust position for the expiration date and put the text
        id_position = (text_position[0], text_position[1] + 20)
        cv2.putText(
            self.image, " (" + str(id) + ")", id_position,
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
        )

    def check_expiration_date(self, expiration):

        # Parse the input string to extract month and year
        try:
            input_date = datetime.strptime(expiration, "%m%Y")
        except ValueError:
            print("Invalid input format. Please provide a valid MMYYYY string.")

        # Get the current date
        current_date = datetime.now()

        # Check if the input date is in the past (expired)
        if input_date < current_date:
            return True
        else:
            return False

    def put_text(self, text):

        # Define the font and size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2

        # Get the size of the text
        text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)

        # Calculate the position for the text (top right corner)
        x = self.image.shape[1] - text_size[0] - 10
        y = text_size[1] + 10

        # Define the color of the text (BGR format)
        text_color = (0, 255, 0)  # White color in BGR

        # Add the text to the image
        cv2.putText(
            self.image, text, (x, y), font, font_scale, text_color,
            font_thickness
        )

    def read_file(self):

        data = {}

        with open(self.file_path, 'r') as csv_file:

            reader = csv.reader(csv_file)
            header = next(reader)  # Read the header row

            # Check if the expected columns are present
            expected_columns = ["ID", "Name", "Expiration"]
            if header != expected_columns:
                rospy.logerr(
                    f"Error: Expected columns {expected_columns}, but found {header}"
                )
                return None

            # Read the data rows
            for row in reader:
                id = int(row[0])
                data[id] = {"name": row[1], "expiration": row[2]}

        return data


if __name__ == "__main__":

    remote_interface = RhInterface()

    while not rospy.is_shutdown():

        remote_interface.main_loop()
        remote_interface.rate.sleep()
