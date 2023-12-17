#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import cv2
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
        self.request_id = 0
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

        rospy.Subscriber(
            '/target_identifier',
            TargetInfo,
            self.__target_identifier_callback,
        ),

        # Service provider
        self.remote_help_service = rospy.Service(
            '/remote_help_request_service',
            UpdateState,
            self.help_request,
        )

        # Service subscriber
        self.resume_task_service = rospy.ServiceProxy(
            '/resume_task',
            UpdateState,
        )

        self.local_help_service = rospy.ServiceProxy(
            '/local_help_request_service',
            UpdateState,
        )

        self.position_fov_service = rospy.ServiceProxy(
            '/calculate_world_position_service',
            ItemPositionFOV,
        )

        self.change_task_state_service = rospy.ServiceProxy(
            '/change_task_state_service',
            UpdateState,
        )
    
    def __target_identifier_callback(self, message):
        
        self.marker_id = message.id
        self.medicine_name = message.name
        self.month = message.expiration[:2]
        self.year = message.expiration[2:]

    def mouse_callback(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.state == 0:

                self.state = 1
                self.rect_start_x, self.rect_start_y = x, y
                self.rect_end_x = None
                self.rect_end_y = None

            if self.state == 2:

                center = Float32MultiArray()
                center.data = [(self.rect_end_x + self.rect_start_x)/2, (self.rect_end_y + self.rect_start_y)/2]

                self.position_fov_service(center)

                if self.is_mouse_inside_button(x, y, self.button_one):
                    # Send to robot
                    self.rh_help = False
                    self.resume_task_service(0)
                    
                    self.change_task_state_service(0)

                    # Update position

                elif self.is_mouse_inside_button(x, y, self.button_two):
                    # Send to LH
                    self.local_help_service(self.request_id)

                self.state = 0

        elif event == cv2.EVENT_LBUTTONUP and self.state == 1:
            
            self.rect_end_x = x
            self.rect_end_y = y

            offset = 10
            self.button_start_x = self.rect_start_x + (self.rect_end_x - self.rect_start_x) + offset
            self.button_start_y = self.rect_start_y

            self.state = 2

        elif event == cv2.EVENT_MOUSEMOVE and self.state == 1:
            self.rect_end_x = x
            self.rect_end_y = y

    def is_mouse_inside_button(self, mouse_x, mouse_y, button_coords):
        
        return (
            button_coords[0][0] < mouse_x < button_coords[1][0] and
            button_coords[0][1] < mouse_y < button_coords[1][1]
        )

    def help_request(self, request):
        print(request.state)
        self.request_id = request.state

        if self.request_id == 0:
            self.rh_help = False
            self.change_task_state_service(0)
        else:
            self.rh_help = True

        self.state = 0
        return True

    def image_callback(self, data):

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.width = data.width
            self.height = data.height

        except CvBridgeError as e:
            print(e)

        self.draw_ar()

        # screen_width, screen_height = self.width*2, self.height*2  # replace with your screen resolution

        # Resize the image or frame to the screen size

        # Create a window with the specified size
        # cv2.resizeWindow("Interface", screen_width, screen_height)

        cv2.imshow("Interface", self.image)
        cv2.setMouseCallback("Interface", self.mouse_callback)
        cv2.waitKey(3)

    def target_position_frame(self, message):

        self.center_x = int(message.data[0])
        self.center_y = int(message.data[1])

        if (self.center_x == 0) and (self.center_y == 0):
            self.target_detected = False
        else:
            self.target_detected = True

    def draw_ar(self):

        # Create an empty image or copy the original image to avoid overwriting the marker
        self.image = self.image.copy()

        self.add_text(
            (int(self.width / 40), int((self.height - (self.height / 100)))),
            "Press A to call local operator",
            (0, 0, 255),
        )

        if self.target_detected:
            cv2.circle(
                self.image, (self.center_x, self.center_y),
                radius=30,
                color=(0, 255, 0),
                thickness=2
            )

        if self.rh_help:
            # If object is not detected
            if self.request_id == 1:
                self.popup_message(1)

            # Medicine is expired
            if self.request_id == 2:

                self.popup_message(2)

            # Object is too big
            if self.request_id == 3:

                self.popup_message(3)

            if (self.rect_start_x is not None) and (self.rect_end_x is not None):
                cv2.rectangle(
                    self.image, (self.rect_start_x, self.rect_start_y),
                    (self.rect_end_x, self.rect_end_y), (0, 255, 0), 2
                )

            if self.state == 2 and (self.button_start_x is not None):
                
                button_rectangle = [(self.button_start_x, self.button_start_y), (self.button_start_x + 120, self.button_start_y + 60)]
                
                self.add_buttons(button_rectangle)


    def add_buttons(self, button_rectangle):

        cv2.rectangle(
            self.image, button_rectangle[0],
            button_rectangle[1], (211, 211, 211), thickness=cv2.FILLED,
        )

        self.button_one = [button_rectangle[0], (button_rectangle[1][0], button_rectangle[0][1] + int((button_rectangle[1][1] - button_rectangle[0][1])/2))]
        cv2.rectangle(
            self.image, self.button_one[0],
            self.button_one[1], (0, 0, 0), 1
        )

        self.button_two = [(self.button_one[0][0], self.button_one[1][1]), button_rectangle[1]]
        cv2.rectangle(
            self.image, self.button_two[0],
            self.button_two[1], (0, 0, 0), 1
        )

        cv2.putText(self.image, "Send to robot", (button_rectangle[0][0] + 5, button_rectangle[0][1] + 20), self.font, 0.5, (0, 0, 0), 1)
        cv2.putText(self.image, "Send to LH", (button_rectangle[0][0] + 5, button_rectangle[0][1] + 50), self.font, 0.5, (0, 0, 0), 1)

    def popup_message(self, case):

        # Draw a red rectangle with border
        cv2.rectangle(
            self.image, self.rect_position, (
                self.rect_position[0] + self.rect_size[0],
                self.rect_position[1] + self.rect_size[1]
            ), self.rect_color, -1
        )
        cv2.rectangle(
            self.image, self.rect_position, (
                self.rect_position[0] + self.rect_size[0],
                self.rect_position[1] + self.rect_size[1]
            ), self.rect_border_color, self.rect_thickness
        )

        self.add_text(
            (self.rect_position[0] + 5, self.rect_position[1] + 25),
            "HELP REQUESTED", (0, 0, 255)
        )

        if case == 1:
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 50),
                "{}".format(self.medicine_name), (0, 0, 0)
            )
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 75),
                "(ID {}) was not".format(self.marker_id), (0, 0, 0)
            )
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 100),
                "detected.", (0, 0, 0)
            )

        if case == 2:
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 50),
                "{}".format(self.medicine_name), (0, 0, 0)
            )
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 75),
                "(ID {}) expired on".format(self.marker_id), (0, 0, 0)
            )
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 100),
                "{}/{}".format(self.month, self.year), (0, 0, 0)
            )

        if case == 3:
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 50),
                "{}".format(self.medicine_name), (0, 0, 0)
            )
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 75),
                "is in the box.".format(self.marker_id), (0, 0, 0)
            )
            self.add_text(
                (self.rect_position[0] + 5, self.rect_position[1] + 100),
                "", (0, 0, 0)
            )

    def add_text(self, position, text, color):

        # Write the text on the image
        cv2.putText(
            self.image, text, position, self.font, self.font_scale, color,
            self.font_thickness
        )

    def on_press(self, key):
        try:
            # Check if the pressed key is 'o'
            if key.char == 'a':
                self.rh_help = False
                self.local_help_service(self.request_id)
                print("yo!")
                print(self.request_id)

            if key.char == 'q':
                self.state = 0

        except AttributeError:
            # Handle special keys if needed
            pass

    def main(self):
        # Set up the listener
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()


if __name__ == "__main__":

    remote_interface = RhInterface()

    while not rospy.is_shutdown():
        remote_interface.main()
        remote_interface.rate.sleep()
