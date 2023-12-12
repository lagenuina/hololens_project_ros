import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class Ar:
    def __init__(self):

        rospy.init_node("interface")
        rospy.Subscriber("shelf_pnp/task_state", Int16, self.subtask_callback)
        rospy.Subscriber("camera/color/image_raw", Image, self.image_callback)

        self.bridge = CvBridge()

        self.font = cv2.FONT_HERSHEY_DUPLEX
        self.font_scale = 0.75
        self.font_thickness = 2

        # Define rectangle properties
        self.rect_position = (5, 5)
        self.rect_size = (200, 175)
        self.rect_color = (255, 255, 244)
        self.rect_border_color = (139, 139, 0)  # Red border color in BGR format
        self.rect_thickness = 2

        self.subtask = None
        
        self.grid = np.array([
        [(191.5, 476), (246, 474), (307, 477), (360.5, 475)],
        [(190, 376), (252.5, 379), (310, 380), (368.5, 380)],
        [(184, 261), (258, 260), (316, 263), (385, 263)],
        [(361, 888), (254.5, 134), (331, 132), (400, 133)],
        ])

        boxes_case1 = [(111.5, 362), (99.5, 248)]
        boxes_case2 = [(124, 460), (111.5, 363)]

        # For each task, set target (column 1) and desired position
        self.tasks = {
            1: (self.grid[0][1],self.grid[2][3]),
            2: (self.grid[2][0], self.grid[0][3]),
            3: (self.grid[1][3], self.grid[0][0], self.grid[0][2]),
            4: (self.grid[3][3], (0,0)),
            5: (self.grid[1][1], self.grid[3][2]),
            6: (self.grid[2][3], (0,0)),
            7: (self.grid[0][3], self.grid[2][1]),
            8: (self.grid[1][2], boxes_case1[0], boxes_case1[1]),
            9: (self.grid[1][0], self.grid[3][1], self.grid[0][0]),
            10: (self.grid[3][1], self.grid[2][2]),
            11: (self.grid[2][1], boxes_case2[0], boxes_case2[1]),
            12: (self.grid[2][2], self.grid[3][0])
        }
        
        self.pnp = [10, 20, 50, 70, 100, 120]
        self.pnp_grasped= [11, 21, 51, 71, 101, 121]
        self.occupied_grasped = [31, 91]
        self.occupied = [30, 90]
        self.unreachable = [40, 60]
        self.boxsearching = [80, 110]

    def subtask_callback(self, msg):
        
        self.subtask = msg.data
        self.key = abs(msg.data) // 10

    def image_callback(self, data):

        try:
           self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image = cv2.rotate(self.image, cv2.ROTATE_90_CLOCKWISE)

        if self.subtask is not None:
            self.draw_ar()

        cv2.imshow("Image", self.image)
        cv2.waitKey(3)

    def draw_ar(self):

        self.image = self.image.copy()

        desired_position = (int(self.tasks[self.key][0][0]), int(self.tasks[self.key][0][1]))
        current_position = (int(self.tasks[self.key][1][0]), int(self.tasks[self.key][1][1]))

        if (self.subtask in self.occupied) or (self.subtask in self.pnp):
            # Draw a red circle in the current position
            self.draw_circle(current_position, 'red')

        elif self.subtask in self.occupied_grasped:

            # Draw a green circle in the desired position
            self.draw_circle(desired_position, 'green')

            # Draw a blue circle for collaborator
            self.draw_circle((int(self.tasks[self.key][2][0]), int(self.tasks[self.key][2][1])), 'blue')
            
            self.popup_message("occupied")
        
        elif self.subtask in self.pnp_grasped:
            # Draw a green circle in the desired position
            self.draw_circle(desired_position, 'green')
            # Draw an arrow pointing at the desired position
            self.draw_arrow(current_position, desired_position)

        elif self.subtask in self.unreachable:
            
            # Draw a green circle in the desired position
            self.draw_circle(desired_position, 'green')            

            self.popup_message("unreachable")

        elif self.subtask in self.boxsearching:

            box1 = current_position
            box2 = (int(self.tasks[self.key][2][0]), int(self.tasks[self.key][2][1]))

            # Draw a red circle in the current position
            cv2.circle(self.image, box1, 60, (0, 0, 255), 3)
            cv2.circle(self.image, box2, 60, (0, 0, 255), 3)

            self.draw_circle(desired_position, 'green')
            
            self.popup_message("box searching")

    def draw_circle(self, position, circle_color):
        
        size = 40
        
        if circle_color == "blue":
            color = (255, 0, 0)
        
        else:
            if circle_color == 'red':
                color = (0, 0, 255)
                text = "Grasp this"

                if self.subtask in self.boxsearching:
                    size = 60

            elif circle_color == 'green':
                color = (0, 255, 0)
                text = "Place here"

            text_position = (position[0] - 60, position[1] - 50)
            self.add_text(text_position, text, color)

        cv2.circle(self.image, position, size, color, 3)
        
    def add_text(self, position, text, color):

        # Write the text on the image
        cv2.putText(self.image, text, position, self.font, self.font_scale, color, self.font_thickness)

    def draw_arrow(self, position1, position2):

        arrow_color = (0, 0, 255)  # Red color for the arrow
        arrow_thickness = 3
        arrowed_line_params = dict(
            tipLength=0.06,
            line_type=cv2.LINE_AA
        )

        # Calculate a new endpoint that is slightly before the circle's center
        offset1 = 40
        offset2 = 80
        direction_vector = np.array(position2) - np.array(position1)
        length = np.linalg.norm(direction_vector)
        normalized_direction = direction_vector / length

        from_position = position1 + offset1 * normalized_direction
        to_position = position2 - offset2 * normalized_direction

        # Convert positions to tuples
        starting_point = tuple(map(int, from_position))
        ending_point = tuple(map(int, to_position))

        cv2.arrowedLine(self.image, starting_point, ending_point, arrow_color, arrow_thickness, **arrowed_line_params)

    def popup_message(self, case):

        # Draw a red rectangle with white border
        cv2.rectangle(self.image, self.rect_position, (self.rect_position[0] + self.rect_size[0], self.rect_position[1] + self.rect_size[1]), self.rect_color, -1)
        cv2.rectangle(self.image, self.rect_position, (self.rect_position[0] + self.rect_size[0], self.rect_position[1] + self.rect_size[1]), self.rect_border_color, self.rect_thickness)

        self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 25), "ASK HELP TO", (0, 0, 0))
        self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 50), "COLLABORATOR", (0, 0, 0))
        
        if case == "occupied":
            self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 90), "Move medicine", (0, 0, 0))
            self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 115), "from green", (0, 255, 0))
            self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 140), "to blue circle", (255, 0, 0))

        elif case == "unreachable":
            if self.subtask == 3:
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 90), "Medicine", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 115), "unreachable!", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 140), "Grab Aspirin", (0, 0, 255))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 165), "500 mg", (0, 0, 255))

            else:
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 90), "Medicine", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 115), "unreachable!", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 140), "Grab Ibuprofen", (0, 0, 255))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 165), "200mg", (0, 0, 255))
            
        elif case == "box searching":
            if self.subtask == 7:
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 90), "Medicine is in", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 115), "a box! Look for", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 140), "Metformin", (0, 0, 255))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 165), "850 mg", (0, 0, 255))
            else:
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 90), "Medicine is in", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 115), "a box! Look for", (0, 0, 0))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 140), "Acetaminophen", (0, 0, 255))
                self.add_text((self.rect_position[0] + 5, self.rect_position[1] + 165), "850 mg", (0, 0, 255))

if __name__ == "__main__":

    ar = Ar()

    while not rospy.is_shutdown():
        pass
