from __future__ import print_function, division

import json
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from iai_robosherlock_msgs.srv import RSQueryService, RSQueryServiceRequest
from rospy import ROSException
from rospy_message_converter import message_converter

from refills_first_review.barcode_detection import BarcodeDetector
from refills_first_review.baseboard_detection import BaseboardDetector
from refills_first_review.separator_detection import SeparatorClustering
from refills_first_review.tfwrapper import TfWrapper

FLOORS = {
    0: [[0, 0, 0.15], [0, 0.1, 0.55], [0, 0.1, 0.88], [0, 0.1, 1.17], [0, 0.1, 1.43]],
    1: [[0, 0, 0.15], [0, 0.1, 0.38], [0, 0.1, 0.59], [0, 0.2, 1.11], [0, 0.2, 1.42]],
    2: [[0, 0, 0.15], [0, 0.1, 0.47], [0, 0.1, 0.76], [0, 0.1, 1.06], [0, 0.1, 1.39]],
    3: [[0, 0, 0.15], [0, 0.1, 0.43], [0, 0.1, 0.68], [0, 0.1, 0.93], [0, 0.1, 1.18], [0, 0.1, 1.43]],
}


class RoboSherlock(object):
    def __init__(self, knowrob):
        self.knowrob = knowrob
        # TODO use paramserver [low]
        # TODO implement all the things [high]

        self.separator_detection = SeparatorClustering(knowrob)
        self.baseboard_detection = BaseboardDetector()
        self.barcode_detection = BarcodeDetector(knowrob)
        try:
            rospy.wait_for_service('/RoboSherlock/json_query', 1)
            self.robosherlock_service = rospy.ServiceProxy('/RoboSherlock/json_query',
                                                           RSQueryService)
            self.robosherlock = True
        except ROSException as e:
            rospy.logwarn('robosherlock not available')
            self.robosherlock = False

        self.tf = TfWrapper()
        rospy.logwarn('robosherlock not fully integrated')

    def start_separator_detection(self, floor_id):
        self.separator_detection.start_listening(floor_id)

    def stop_separator_detection(self):
        return self.separator_detection.stop_listening()

    def start_baseboard_detection(self):
        self.baseboard_detection.start_listening()

    def stop_baseboard_detection(self):
        return self.baseboard_detection.stop_listening()

    def start_barcode_detection(self, shelf_id, floor_id):
        self.barcode_detection.start_listening(shelf_id, floor_id)
        pass

    def stop_barcode_detection(self):
        return self.barcode_detection.stop_listening()

    def detect_floors(self, shelf_id):
        # TODO
        rospy.loginfo('detecting floors for shelf {}'.format(shelf_id))
        rospy.logwarn('floor detection not implemented')
        return FLOORS[shelf_id]

    def start_floor_detection(self, shelf_id):
        if self.robosherlock:
            req = RSQueryServiceRequest()
            q = {"scan":
                     {"type": "shelf",
                      "location": shelf_id,
                      "command": "start"}}
            req.query = json.dumps(q)
            self.robosherlock_service.call(req)

    def stop_floor_detection(self, shelf_id, unofficial_shelf_id=0):
        if self.robosherlock:
            req = RSQueryServiceRequest()
            q = {'scan':
                     {'type': 'shelf',
                      'location': '',
                      'command': 'stop'}}
            req.query = json.dumps(q)
            result = self.robosherlock_service.call(req)
            floors = []
            for floor in result.answer:
                p = message_converter.convert_dictionary_to_ros_message('geometry_msgs/PoseStamped',
                                                                        json.loads(floor)['poses'][0]['pose_stamped'])
                p = self.tf.transform_pose(shelf_id, p)
                floors.append([0,
                               p.pose.position.y,
                               p.pose.position.z])
            floors = list(sorted(floors, key=lambda x: x[-1]))
            floors = [x for x in floors if x[-1] > 0.3]
            floors = [FLOORS[shelf_id][0]] + floors
            print('detected shelfs at heights: {}'.format(floors))
        else:
            floors = FLOORS[unofficial_shelf_id]
        return floors

    def count(self):
        # TODO
        return int(np.random.random() * 4)


if __name__ == '__main__':
    rospy.init_node('muh')
    rs = RoboSherlock()
    shelf_id = 'shelf_system_0'
    rs.start_floor_detection(shelf_id)
    cmd = raw_input('stop? [enter]')
    print(rs.stop_floor_detection('shelf_system_0'))
