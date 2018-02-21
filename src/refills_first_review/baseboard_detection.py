from __future__ import division

from simplejson import OrderedDict

import rospy
import numpy as np

from copy import deepcopy

from collections import defaultdict
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion, Pose
from rospy import ROSException
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_matrix
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from refills_first_review.tfwrapper import TfWrapper

MAP = 'map'


# TODO use compressed topic [medium]

class Shelf(object):
    def __init__(self, number, map_position):
        self.complete = False
        self.id = self.id_from_number(number)
        rospy.loginfo('added shelf {}'.format(self.id))
        self.left_measurements = []
        self.right_measurements = []
        self.add_measurement(number, map_position)

    def add_measurement(self, number, map_position):
        if self.id != self.id_from_number(number):
            return False
        if self.is_left(number):
            self.left_measurements.append(map_position)
        else:
            self.right_measurements.append(map_position)
        self.is_complete()
        return True

    def is_left(self, number):
        return number % 2 == 0

    def id_from_number(self, number):
        if number % 2 != 0:
            number -= 1
        number /= 2
        number -= 10
        return int(number)

    def is_complete(self):
        if not self.complete and len(self.left_measurements) > 0 and len(self.right_measurements) > 0:
            self.complete = True
            rospy.loginfo('shelf {} complete'.format(self.id))
        return self.complete

    def get_left(self):
        return np.mean(self.left_measurements, axis=0)

    def get_right(self):
        return np.mean(self.right_measurements, axis=0)

    def get_shelf(self):
        p = PoseStamped()
        p.header.frame_id = MAP
        p.pose.position = Point(*self.calc_shelf_origin())
        p.pose.orientation = Quaternion(*self.get_orientation())
        return 'shelf{}'.format(self.id), p

    def calc_shelf_origin(self):
        left = self.get_left()
        right = self.get_right()
        a = left - right
        a_length = np.linalg.norm(a)
        a /= a_length
        a *= (1 - a_length) / 2
        origin = left + a
        origin[2] = 0
        return origin

    def get_orientation(self):
        left = self.get_left()
        right = self.get_right()
        y = right - left
        y = y / np.linalg.norm(y)
        z = [0, 0, 1]
        x = np.cross(y, z)
        x = x / np.linalg.norm(x)
        return quaternion_from_matrix([
            [x[0], x[1], x[2], 0],
            [y[0], y[1], y[2], 0],
            [z[0], z[1], z[2], 0],
            [0.0, 0.0, 0.0, 1.0],
        ])


class BaseboardDetector(object):
    def __init__(self):
        # TODO use paramserver [low]
        self.shelf_width = 1

        self.tf = TfWrapper()
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.marker_ns = 'baseboard_marker'

        self.shelves = []
        self.baseboard_detector_topic = '/ros_markers/tf'
        try:
            rospy.wait_for_message('/refills_wrist_camera/image_color', rospy.AnyMsg, timeout=1)
        except ROSException as e:
            rospy.loginfo('camera offline; \'detecting\' shelves anyway')
            self.detect_fake_shelves()

        self.left_color = ColorRGBA(1, 1, 0, 1)
        self.right_color = ColorRGBA(1, .5, 0, 1)

    def start_listening(self):
        self.detector_sub = rospy.Subscriber(self.baseboard_detector_topic, TFMessage, self.cb, queue_size=10)

    def stop_listening(self):
        self.detector_sub.unregister()
        self.publish_as_marker()
        return OrderedDict([x.get_shelf() for x in self.shelves if x.is_complete()])

    def detect_fake_shelves(self):
        s1 = Shelf(20, [-0.05, 0.515, 0])
        s1.add_measurement(21, [-0.95, 0.515, 0])
        self.shelves.append(s1)
        s2 = Shelf(22, [-1.05, 0.515, 0])
        s2.add_measurement(23, [-1.95, 0.515, 0])
        self.shelves.append(s2)
        s3 = Shelf(24, [-2.05, 0.515, 0])
        s3.add_measurement(25, [-2.95, 0.515, 0])
        self.shelves.append(s3)
        s4 = Shelf(26, [-3.05, 0.515, 0])
        s4.add_measurement(27, [-3.95, 0.515, 0])
        self.shelves.append(s4)

    def cb(self, data):
        for msg in data.transforms:
            number = int(msg.child_frame_id.split('_')[-1])
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = Point(msg.transform.translation.x,
                                       msg.transform.translation.y,
                                       msg.transform.translation.z)
            pose.pose.orientation.w = 1
            pose = self.tf.transformPose(MAP, pose)
            position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for shelf in self.shelves:
                if shelf.add_measurement(number, position):
                    break
            else:  # if not break
                self.shelves.append(Shelf(number, position))

    def publish_as_marker(self):
        ma = MarkerArray()
        for i, shelf in enumerate(self.shelves):
            if shelf.is_complete():
                # left
                m = Marker()
                m.header.frame_id = MAP
                m.ns = self.marker_ns
                m.id = i
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position = Point(*shelf.get_left())
                m.pose.position.z = 0.03
                m.pose.orientation = Quaternion(*shelf.get_orientation())
                m.scale = Vector3(.03, .05, .05)
                m.color = self.left_color
                ma.markers.append(m)
                # right
                m = deepcopy(m)
                m.id += 900000
                m.pose.position = Point(*shelf.get_right())
                m.pose.position.z = 0.03
                m.color = self.right_color
                ma.markers.append(m)
        self.marker_pub.publish(ma)


if __name__ == '__main__':
    rospy.init_node('baseboard_detection_test')
    d = BaseboardDetector()
    d.start_listening()
    print('baseboard detection test started')
    cmd = raw_input('stop? [enter]')
    print('baseboard detection test ended')
    print(d.stop_listening())
    rospy.sleep(.5)
