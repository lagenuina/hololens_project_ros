#!/usr/bin/env python
import rospy
import cv2
import csv
from datetime import datetime
from cv_bridge import (CvBridge, CvBridgeError)
from std_msgs.msg import (Float32MultiArray, Int32)
from sensor_msgs.msg import (Image)

class RhInterface:

    def __init__(self):

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__BRIDGE = CvBridge()
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )
        aruco_params = cv2.aruco.DetectorParameters()
        self.__ARUCO_DETECTOR = cv2.aruco.ArucoDetector(
            aruco_dict,
            aruco_params,
        )
        self.__FILE_PATH = '/home/fetch/catkin_workspaces/hololens_ws/src/holo_project/src/scripts/csv files/id_medicine.csv'
        self.__OBJECTS_INFO = self.__read_file()

        # # Public CONSTANTS:
        self.RATE = rospy.Rate(100)

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__image = None
        self.__object_center = []
        self.__target_detected = False
        self.__state = 0

        self.__last_seen_markers = {}

        # # Topic publisher:
        self.__image_pub = rospy.Publisher(
            "/chest_cam/remote_interface",
            Image,
            queue_size=1,
        )     

        # # Topic subscriber:
        rospy.Subscriber(
            '/chest_cam/camera/color/image_raw',
            Image,
            self.__image_callback,
        ),

        rospy.Subscriber(
            '/workspace_cam/target_position_in_frame',
            Float32MultiArray,
            self.__object_center_callback,
        ),

        rospy.Subscriber(
            '/my_gen3/pick_and_place',
            Int32,
            self.__robot_state_callback,
        )

    def __image_callback(self, message):

        try:
            self.__image = self.__BRIDGE.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def __object_center_callback(self, message):

        self.__object_center = [int(message.data[0]), int(message.data[1])]

        self.__target_detected = self.__object_center != [0, 0]

        # if self.__object_center != [0, 0]:
        #     self.__target_detected = False
        # else:
        #     self.__target_detected = True

    def __robot_state_callback(self, message):

        self.__state = message.data

    def __dispay_information(self, corner, name, id, expiration):

        # Calculate the position for the text
        text_position = (int(corner[0][0] - 30), int(corner[0][1] - 80))

        if self.__check_expiration_date(expiration):
            color = (0, 40, 255)
        else:
            color = (255, 150, 0)

        # Put the name of the medicine
        cv2.putText(
            self.__image, name, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            color, 1
        )

        # Adjust position for the expiration date and put the text
        id_position = (text_position[0], text_position[1] + 20)
        cv2.putText(
            self.__image, " (" + str(id) + ")", id_position,
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
        )

    def __check_expiration_date(self, expiration):

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

    def __put_text(self, text):

        # Define the font and size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2

        # Get the size of the text
        text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)

        # Calculate the position for the text (top right corner)
        x = self.__image.shape[1] - text_size[0] - 10
        y = text_size[1] + 10

        # Define the color of the text (BGR format)
        text_color = (0, 255, 0)  # White color in BGR

        # Add the text to the image
        cv2.putText(
            self.__image, text, (x, y), font, font_scale, text_color,
            font_thickness
        )

    def __read_file(self):

        data = {}

        try:
            with open(self.__FILE_PATH, 'r') as csv_file:

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
        except Exception as e:
            rospy.logerr(f"Failed to read CSV file: {e}")

        return data

    def __main_loop(self):

        if self.__image is not None:
            self.__image = self.__image.copy()

            if self.__state != 0:

                if self.__state == 1:
                    text = "Grasping"
                    if self.__target_detected:
                        cv2.circle(
                            self.__image, (self.__object_center[0], self.__object_center[1]),
                            radius=30,
                            color=(0, 255, 0),
                            thickness=2,
                        )
                elif self.__state == 2:
                    text = "Grasped"
                elif self.__state == 3:
                    text = "Placing"
                elif self.__state == 4:
                    text = "Placed"

                self.__put_text(text)

            else:
                corners, ids, _ = self.__ARUCO_DETECTOR.detectMarkers(self.__image)
                current_time = rospy.Time.now()

                detected_ids = set()

                # Process detected markers
                if ids is not None:
                    for i, corner in enumerate(corners):
                        id = ids[i][0]
                        detected_ids.add(id)
                        if id in self.__OBJECTS_INFO:
                            object_info = self.__OBJECTS_INFO[id]
                            name = object_info["name"]
                            expiration = object_info["expiration"]

                            # Update last seen position and time
                            self.__last_seen_markers[id] = {
                                'corner': corner[0],
                                'time': current_time,
                                'name': name,
                                'expiration': expiration
                            }

                            # Display information for detected marker
                            self.__dispay_information(
                                corner[0], name, id, expiration
                            )

                # Check for markers that were not detected in this frame
                for id, info in self.__last_seen_markers.items():
                    if id not in detected_ids and (current_time
                                                   - info['time']).to_sec() < 3:
                        # Marker was not detected but should still be displayed
                        self.__dispay_information(
                            info['corner'],
                            info['name'],
                            id,
                            info['expiration'],
                        )

            image = Image()
            image = self.__BRIDGE.cv2_to_imgmsg(
                self.__image,
                encoding="bgr8",
            )
            self.__image_pub.publish(image)

def main():
    """
    """

    rospy.init_node(
        'remote_interface',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    remote_interface = RhInterface()

    while not rospy.is_shutdown():

        remote_interface.__main_loop()
        remote_interface.RATE.sleep()

if __name__ == "__main__":

    main()