import rospy
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
import math
import tf


class MobileBaseController:

    def __init__(self):

        rospy.init_node('mobile_base_controller_node', anonymous=True)

        self.current_orientation = None

        self.position_base = None
        self.rotation_base = None
        self.target_position = None

        self.target_reached = False

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        rospy.Subscriber('/target_pose', Point, self.set_target)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz publishing rate

    def update_transform(self):

        try:
            #TF from spatial anchor to robot base
            (self.position_base,
             self.rotation_base) = self.listener.lookupTransform(
                 '/4862383b-950c-4782-96b7-15c9d726c00b', '/base_link',
                 rospy.Time(0)
             )
            self.roll, self.pitch, self.yaw = euler_from_quaternion(
                self.rotation_base
            )
            self.current_orientation = self.roll
            print(self.roll, self.pitch, self.yaw)
            # print(self.position_base, self.current_orientation)

        except (
            tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException
        ):
            self.rate.sleep()

    def set_target(self, msg):

        self.target_position = [msg.x, msg.y, -msg.z]

        self.update_transform()
        # self.br.sendTransform((self.target_position[0], self.target_position[1], self.target_position[2]),
        # (0.0, 0.0, 0.0, 1.0),
        # rospy.Time.now(),
        # 'target',
        # '4862383b-950c-4782-96b7-15c9d726c00b')

        # while self.position_base is None:
        # 	self.update_transform()

    def calculate_distance(self):
        distance = math.sqrt(
            (self.target_position[0] - self.position_base[0])**2
            + (self.target_position[1] - self.position_base[1])**2
            + (self.target_position[2] - self.position_base[2])**2
        )
        # distance = math.sqrt((self.position_base[0])**2 + (self.position_base[1])**2 + (self.position_base[2])**2)
        return distance

    def calculate_rotation(self):

        target_angle = -(
            math.atan2(
                self.target_position[0] - self.position_base[0],
                self.target_position[2] - self.position_base[2]
            )
        )

        rotation = target_angle - self.current_orientation

        print("target angle", target_angle)
        print("current rotation", self.current_orientation)
        # Normalize angle to -pi to pi range
        rotation = -math.atan2(math.sin(rotation), math.cos(rotation))
        print("rotation", rotation)

        if abs(rotation) > 0.2:
            return rotation * 0.5
        else:
            return 0

        # if rotation > 0.2:
        # 	return 0.1
        # elif rotation < 0.2:
        # 	return -0.1
        # else:
        # 	return 0

    def move_to_target(self):
        if self.position_base is not None:

            while not self.target_reached:

                self.update_transform()

                distance = self.calculate_distance()
                # print(distance)

                if distance < 0.5:  # Set a threshold for target proximity
                    self.target_reached = True
                    rospy.loginfo('Target reached!')
                    break

                linear_vel = 0.1
                angular_vel = self.calculate_rotation() * 0.5

                twist_msg = Twist()
                twist_msg.linear.x = linear_vel
                # twist_msg.angular.z = 0.0
                twist_msg.angular.z = angular_vel

                self.vel_pub.publish(twist_msg)
                self.rate.sleep()

            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

            self.vel_pub.publish(twist_msg)


if __name__ == '__main__':

    controller = MobileBaseController()

    while not rospy.is_shutdown():
        controller.move_to_target()
