#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import Int32, Bool
from Scripts.msg import TargetInfo
from Scripts.srv import BoolUpdate
from gopher_ros_clearcore.msg import (Position)


class FileReader:

    def __init__(self):

        self.file_path = '/home/fetch/catkin_workspaces/hololens_ws/src/holo_project/src/scripts/task_sequence.csv'

        self.csv_data = self.read_file()
        self.rate = rospy.Rate(1)

        self.on_startup = True
        self.adjusting_chest = False
        self.target_pub = rospy.Publisher(
            '/target_identifier',
            TargetInfo,
            queue_size=1,
        )

        # self.medicine_pub = rospy.Publisher(
        #     '/medicine_name',
        #     String,
        #     queue_size=1,
        # )

        # self.medicine_id_pub = rospy.Publisher(
        #     '/medicine_id',
        #     Int32,
        #     queue_size=1,
        # )

        # self.__chest_position = rospy.Publisher(
        #     'z_chest_pos',
        #     Position,
        #     queue_size=1,
        # )

        self.target_counter_pub = rospy.Publisher(
            '/target_counter',
            Int32,
            queue_size=1,
        )

        # self.adjusting_chest_pub = rospy.Publisher(
        #     '/adjusting_chest',
        #     Bool,
        #     queue_size=1,
        # )

        self.counter = -1
        self.update = False

        self.chest_position = 440
        self.update_target_service = rospy.Service(
            '/update_target', BoolUpdate, self.update_target
        )

    def update_target(self, request):

        self.update = request.data
        self.counter += 1

        # if self.counter == 0 and self.on_startup:

        #     while self.chest_position > 200.0:
        #         print(self.chest_position)
        #         chest_pos = Position()
        #         self.chest_position -= 10
        #         chest_pos.position = self.chest_position
        #         chest_pos.velocity = 1.0
        #         self.__chest_position.publish(chest_pos)

        #         self.adjusting_chest = True

        #         # rospy.sleep(1)

        # self.on_startup = False
        # self.adjusting_chest = False

        return True

    def read_file(self):

        data = []

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
                entry = {
                    "id": int(row[0]),
                    "name": row[1],
                    "expiration": row[2],
                }
                data.append(entry)

        return data

    def main_loop(self):

        if self.update and self.counter < len(self.csv_data):

            # if not self.on_startup:
            #     print(self.on_startup)
            target_counter = Int32()
            target_counter.data = self.counter
            self.target_counter_pub.publish(target_counter)

            if self.counter >= 0:
                new_target = TargetInfo()
                new_target.id = int(self.csv_data[self.counter]['id'])
                new_target.name = self.csv_data[self.counter]['name']
                new_target.expiration = self.csv_data[self.counter
                                                     ]['expiration']

                self.target_pub.publish(new_target)

            # chest_status = Bool()
            # chest_status.data = self.adjusting_chest
            # self.adjusting_chest_pub.publish(chest_status)

            # medicine_name = String()
            # medicine_name.data = self.csv_data[self.counter]['name']

            # self.medicine_pub.publish(medicine_name)

            # medicine_id = Int32()
            # medicine_id.data = int(self.csv_data[self.counter]['id'])

            # self.medicine_id_pub.publish(medicine_id)


if __name__ == '__main__':

    rospy.init_node('csv_reader_node', anonymous=True)
    reader = FileReader()

    while not rospy.is_shutdown():
        reader.main_loop()
        reader.rate.sleep()
