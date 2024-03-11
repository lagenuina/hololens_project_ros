#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import (Int32)
from Scripts.msg import (TargetInfo)
from Scripts.srv import (BoolUpdate)
from std_srvs.srv import (Empty)


class TaskStateManager:

    def __init__(
        self,
        node_name,
        task,
    ):

        # # Private CONSTANTS:
        self.__NODE_NAME = node_name

        # # Public CONSTANTS:
        self.RATE = rospy.Rate(1)
        self.TASK = task

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__counter = -2
        self.__task_started = True
        self.__task_ended = False
        self.__update = False

        # # Service provider:
        rospy.Service(
            '/update_target',
            Empty,
            self.__update_target,
        )

        # # Topic publisher:
        self.__target_identifier = rospy.Publisher(
            '/target_identifier',
            TargetInfo,
            queue_size=1,
        )

        self.__target_counter = rospy.Publisher(
            '/target_counter',
            Int32,
            queue_size=1,
        )

        if self.TASK == 'study':

            csv_file = 'task_sequence.csv'

            rospy.wait_for_service('/data_writer/resume_recording')

            self.__start_image_recording = rospy.ServiceProxy(
                '/chest_cam/image_writer/resume_recording',
                Empty,
            )

            self.__stop_image_recording = rospy.ServiceProxy(
                '/chest_cam/image_writer/finish_recording',
                Empty,
            )

            self.__start_recording = rospy.ServiceProxy(
                '/data_writer/resume_recording',
                Empty,
            )

            self.__stop_recording = rospy.ServiceProxy(
                '/data_writer/finish_recording',
                Empty,
            )

        elif self.TASK == 'training':
            csv_file = 'task_sequence_training.csv'

        self.__file_path = '/home/fetch/catkin_workspaces/hololens_ws/src/holo_project/src/scripts/csv files/' + csv_file

        self.__csv_data = self.__read_file()

    def __update_target(self, request):

        self.__update = request.data
        self.__counter += 1

        return []

    def __read_file(self):

        data = []

        with open(self.__file_path, 'r') as csv_file:

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

        if self.__update and self.__counter < len(self.__csv_data):

            if self.__task_started and self.__counter == 0:

                if self.TASK == 'study':
                    self.__start_recording()
                    self.__start_image_recording()

                self.__task_started = False

            target_counter = Int32()
            target_counter.data = self.__counter
            self.__target_counter.publish(target_counter)

            if self.__counter >= 0:
                new_target = TargetInfo()
                new_target.id = int(self.__csv_data[self.__counter]['id'])
                new_target.name = self.__csv_data[self.__counter]['name']
                new_target.expiration = self.__csv_data[self.__counter
                                                       ]['expiration']

                self.__target_identifier.publish(new_target)

        if self.__counter == len(self.__csv_data) and not self.__task_ended:

            if self.TASK == 'study':
                self.__stop_recording()
                self.__stop_image_recording()

            self.__task_ended = True


def main():
    """
    """

    rospy.init_node(
        'task_manager',
        log_level=rospy.INFO,
    )

    rospy.loginfo('\n\n\n\n\n')

    # # ROS parameters:
    current_task = rospy.get_param(
        param_name=f'{rospy.get_name()}/task',
        default='study',
    )

    task_manager = TaskStateManager(node_name='task_manager', task=current_task)

    while not rospy.is_shutdown():
        task_manager.main_loop()
        task_manager.RATE.sleep()


if __name__ == '__main__':
    main()
