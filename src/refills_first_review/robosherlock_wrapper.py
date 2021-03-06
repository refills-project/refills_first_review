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
from refills_first_review.tfwrapper import transform_pose, lookup_transform

FLOORS = {
    0: [0.15, 0.55, 0.88, 1.17, 1.43],
    1: [0.15, 0.38, 0.59, 1.11, 1.42],
    2: [0.15, 0.47, 0.76, 1.06, 1.39],
    3: [0.15, 0.43, 0.68, 0.93, 1.18, 1.43],
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
        self.ring_light_srv = rospy.ServiceProxy('iai_ringlight_controller', iai_ringlight_in)
        self.floor_detection = True
        self.counting = True
        try:
            rospy.wait_for_service('/RoboSherlock/query', 1)
            self.robosherlock_service = rospy.ServiceProxy('/RoboSherlock/query',
                                                           RSQueryService)
            self.robosherlock = True
        except ROSException as e:
            rospy.logwarn('robosherlock not available; activating fake perception')
            self.robosherlock = False

        self.set_ring_light(True)

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
        self.set_ring_light(True)
        if self.robosherlock and self.floor_detection:
            req = RSQueryServiceRequest()
            q = {"scan":
                     {"type": "shelf",
                      "location": self.knowrob.get_perceived_frame_id(shelf_id),
                      "command": "start"}}
            req.query = json.dumps(q)
            print('robosherlock_service.call(req) START 1')
            self.robosherlock_service.call(req)
            print('robosherlock_service.call(req) END 1')

    def stop_floor_detection(self, shelf_id):
        shelf_frame = self.knowrob.get_perceived_frame_id(shelf_id)
        if self.robosherlock and self.floor_detection:
            req = RSQueryServiceRequest()
            q = {'scan':
                     {'type': 'shelf',
                      'location': shelf_frame,
                      'command': 'stop'}}
            req.query = json.dumps(q)
            print('robosherlock_service.call(req) START 2')

            result = self.robosherlock_service.call(req)
            print('robosherlock_service.call(req) END 2')
            floors = []
            for floor in result.answer:
                p = message_converter.convert_dictionary_to_ros_message('geometry_msgs/PoseStamped',
                                                                        json.loads(floor)['poses'][0]['pose_stamped'])
                p = transform_pose(shelf_frame, p)
                floors.append(p.pose.position.z)
            floors = list(sorted(floors))
            # floors = [x for x in floors if x[-1] > 0.3]
            # floors = [FLOORS[0][0]] + floors
            print('detected shelfs at heights: {}'.format(floors))

            # TODO remove this if floor detection works
            # shelf_pose = lookup_transform(MAP, shelf_frame)
            # floors = FLOORS[int(shelf_pose.pose.position.x)]
        else:
            shelf_pose = lookup_transform(MAP, shelf_frame)
            floors = FLOORS[int(shelf_pose.pose.position.x)]
        floors = self.merge_close_shelf_layer(floors)
        return floors

    def merge_close_shelf_layer(self, layer_heights):
        result = []
        for e in layer_heights:
            if len(result) == 0:
                result.append(e)
            elif abs(e - result[-1]) > 0.07:
                result.append(e)
            else:
                result[-1] = (e + result[-1]) / 2
        return result

    def count(self, facing_id, perceived_shelf_frame_id):
        self.set_ring_light(True)
        if self.robosherlock and self.counting:
            # ls = lookup_transform(perceived_shelf_frame_id,
            #                               self.knowrob.get_perceived_frame_id(left_separator))
            # q = {'detect':{
            #     'type': product,
            #     'pose_stamped': convert_ros_message_to_dictionary(ls),
            #     'shelf_type': facing_type,
            #     'width': width,
            #     'location': perceived_shelf_frame_id
            # }}

            q = {'detect': {
                'facing': facing_id,
                'location': perceived_shelf_frame_id
            }}
            print(q)
            req = RSQueryServiceRequest()
            req.query = json.dumps(q)
            rospy.sleep(0.4)
            print('robosherlock_service.call(req) START 3')
            result = self.robosherlock_service.call(req)
            print('robosherlock_service.call(req) END 3')
            print(result)
            count = len(result.answer)
        else:
            count = int(np.random.random() * 4)
        # self.set_ring_light(True)
        return count
        # return 1

# if __name__ == '__main__':
#     rospy.init_node('muh')
#     rs = RoboSherlock()
#     shelf_id = 'shelf_system_0'
#     rs.start_floor_detection(shelf_id)
#     cmd = raw_input('stop? [enter]')
#     print(rs.stop_floor_detection('shelf_system_0'))
