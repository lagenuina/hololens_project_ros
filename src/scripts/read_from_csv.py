#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import Int32
from Scripts.msg import TargetInfo
from Scripts.srv import BoolUpdate, UpdateState
from std_srvs.srv import Empty


class FileReader:

    def __init__(
        self,
        task,
    ):

        self.task = task

        if self.task == 'study':
            csv_file = 'task_sequence.csv'

            rospy.wait_for_service('/data_writer/resume_recording')

            self.start_image_recording = rospy.ServiceProxy(
                '/chest_cam/image_writer/resume_recording',
                Empty,
            )

            self.stop_image_recording = rospy.ServiceProxy(
                '/chest_cam/image_writer/finish_recording',
                Empty,
            )

            self.start_recording = rospy.ServiceProxy(
                '/data_writer/resume_recording',
                Empty,
            )

            self.stop_recording = rospy.ServiceProxy(
                '/data_writer/finish_recording',
                Empty,
            )

        elif self.task == 'training':
            csv_file = 'task_sequence_training.csv'

        self.file_path = '/home/fetch/catkin_workspaces/hololens_ws/src/holo_project/src/scripts/' + csv_file

        self.csv_data = self.read_file()
        self.rate = rospy.Rate(1)

        self.task_started = True
        self.task_ended = False

        self.adjusting_chest = False
        self.target_pub = rospy.Publisher(
            '/target_identifier',
            TargetInfo,
            queue_size=1,
        )

        self.target_counter_pub = rospy.Publisher(
            '/target_counter',
            Int32,
            queue_size=1,
        )

        self.counter = -2
        self.update = False

        self.chest_position = 440
        self.update_target_service = rospy.Service(
            '/update_target',
            BoolUpdate,
            self.update_target,
        )

    def update_target(self, request):

        self.update = request.data
        self.counter += 1

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

            if self.task_started and self.counter == 0:

                if self.task == 'study':
                    self.start_recording()
                    self.start_image_recording()

                self.task_started = False

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

        if self.counter == len(self.csv_data) and not self.task_ended:

            if self.task == 'study':
                self.stop_recording()
                self.stop_image_recording()

            self.task_ended = True


if __name__ == '__main__':

    rospy.init_node('csv_reader_node', anonymous=True)

    # # ROS parameters:
    task = rospy.get_param(
        param_name=f'{rospy.get_name()}/task',
        default='study',
    )
    reader = FileReader(task=task)

    while not rospy.is_shutdown():
        reader.main_loop()
        reader.rate.sleep()
