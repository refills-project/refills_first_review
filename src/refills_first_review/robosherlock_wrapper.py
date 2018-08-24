from __future__ import print_function, division

import json
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from iai_ringlight.srv import iai_ringlight_in, iai_ringlight_inRequest
from iai_robosherlock_msgs.srv import RSQueryService, RSQueryServiceRequest
from rospy import ROSException
from rospy_message_converter import message_converter
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
from std_srvs.srv import SetBool, SetBoolRequest

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

MAP = 'map'

class RoboSherlock(object):
    def __init__(self, knowrob):
        self.knowrob = knowrob
        # TODO use paramserver [low]
        # TODO implement all the things [high]

        self.separator_detection = SeparatorClustering(knowrob)
        self.baseboard_detection = BaseboardDetector()
        self.barcode_detection = BarcodeDetector(knowrob)
        self.ring_light_srv = rospy.ServiceProxy('IAI_ringlight_controller', iai_ringlight_in)
        self.floor_detection = True
        try:
            rospy.wait_for_service('/RoboSherlock/json_query', 1)
            self.robosherlock_service = rospy.ServiceProxy('/RoboSherlock/json_query',
                                                           RSQueryService)
            self.robosherlock = True
        except ROSException as e:
            rospy.logwarn('robosherlock not available; activating fake perception')
            self.robosherlock = False

        self.tf = TfWrapper()

    def set_ring_light(self, value=True):
        rospy.loginfo('calling ring light switch')
        if value:
            req = iai_ringlight_inRequest(a=9)
        else:
            req = iai_ringlight_inRequest(a=0)
        r = None
        try:
            r = self.ring_light_srv.call(req)
        except:
            rospy.logwarn('ring_light_switch not available')
        rospy.loginfo('ring light switch returned {}'.format(r))

    def start_separator_detection(self, floor_id):
        self.set_ring_light(True)
        self.separator_detection.start_listening_separators(floor_id)

    def start_mounting_bar_detection(self, floor_id):
        self.set_ring_light(True)
        self.separator_detection.start_listening_mounting_bars(floor_id)

    def stop_separator_detection(self):
        return self.separator_detection.stop_listening()

    def start_baseboard_detection(self):
        self.set_ring_light(True)
        self.baseboard_detection.start_listening()

    def stop_baseboard_detection(self):
        return self.baseboard_detection.stop_listening()

    def start_barcode_detection(self, shelf_id, floor_id):
        self.set_ring_light(True)
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
        self.set_ring_light(False)
        if self.robosherlock and self.floor_detection:
            req = RSQueryServiceRequest()
            q = {"scan":
                     {"type": "shelf",
                      "location": self.knowrob.get_perceived_frame_id(shelf_id),
                      "command": "start"}}
            req.query = json.dumps(q)
            self.robosherlock_service.call(req)

    def stop_floor_detection(self, shelf_id):
        shelf_frame = self.knowrob.get_perceived_frame_id(shelf_id)
        if self.robosherlock and self.floor_detection:
            req = RSQueryServiceRequest()
            q = {'scan':
                     {'type': 'shelf',
                      'location': shelf_frame,
                      'command': 'stop'}}
            req.query = json.dumps(q)
            result = self.robosherlock_service.call(req)
            floors = []
            for floor in result.answer:
                p = message_converter.convert_dictionary_to_ros_message('geometry_msgs/PoseStamped',
                                                                        json.loads(floor)['poses'][0]['pose_stamped'])
                p = self.tf.transform_pose(shelf_frame, p)
                floors.append([0,
                               p.pose.position.y,
                               p.pose.position.z])
            floors = list(sorted(floors, key=lambda x: x[-1]))
            floors = [x for x in floors if x[-1] > 0.3]
            floors = [FLOORS[0][0]] + floors
            print('detected shelfs at heights: {}'.format(floors))

            # TODO remove this if floor detection works
            shelf_pose = self.tf.lookup_transform(MAP, shelf_frame)
            floors = FLOORS[int(shelf_pose.pose.position.x)]
        else:
            shelf_pose = self.tf.lookup_transform(MAP, shelf_frame)
            floors = FLOORS[int(shelf_pose.pose.position.x)]
        return floors

    def count(self, product, width, left_separator, perceived_shelf_frame_id, facing_type='standing'):
        self.set_ring_light(False)
        if self.robosherlock:
            ls = self.tf.lookup_transform(perceived_shelf_frame_id,
                                          self.knowrob.get_perceived_frame_id(left_separator))
            q = {'detect':{
                'type': product,
                'pose_stamped': convert_ros_message_to_dictionary(ls),
                'shelf_type': facing_type,
                'width': width,
                'location': perceived_shelf_frame_id
            }}
            print(q)
            req = RSQueryServiceRequest()
            req.query = json.dumps(q)
            rospy.sleep(0.4)
            result = self.robosherlock_service.call(req)
            print(result)
            count = len(result.answer)
        else:
            count = int(np.random.random() * 4)
        # self.set_ring_light(True)
        return count


# if __name__ == '__main__':
#     rospy.init_node('muh')
#     rs = RoboSherlock()
#     shelf_id = 'shelf_system_0'
#     rs.start_floor_detection(shelf_id)
#     cmd = raw_input('stop? [enter]')
#     print(rs.stop_floor_detection('shelf_system_0'))
