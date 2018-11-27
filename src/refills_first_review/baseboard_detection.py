from __future__ import division

from simplejson import OrderedDict

import rospy
import numpy as np

from copy import deepcopy

from collections import defaultdict
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion, Pose
from rospy import ROSException
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_matrix, quaternion_from_euler, quaternion_about_axis
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from refills_first_review.tfwrapper import transform_pose

MAP = 'map'

MARKER_OFFSET = 0.025


class Shelf(object):
    def __init__(self, number, map_position):
        self.complete = False
        self.id = self.id_from_number(number)
        rospy.loginfo('added shelf system {}'.format(self.id))
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
        return number % 2 == 1

    def id_from_number(self, number):
        if not self.is_left(number):
            number -= 1
        # number /= 2
        # number -= 10
        return int(number)

    def is_complete(self):
        if not self.complete and len(self.left_measurements) > 0 and len(self.right_measurements) > 0:
            self.complete = True
            rospy.loginfo('shelf system {} complete'.format(self.id))
        return self.complete

    def get_left(self):
        left = np.mean(self.left_measurements, axis=0)
        return left

    def get_right(self):
        return np.mean(self.right_measurements, axis=0)

    def get_shelf(self):
        p = PoseStamped()
        p.header.frame_id = MAP
        p.pose.position = Point(*self.calc_shelf_origin())
        p.pose.orientation = Quaternion(*self.get_orientation())
        return self.get_name(), p

    def calc_shelf_origin(self):
        left = self.get_left()
        right = self.get_right()
        a = left - right
        a_length = np.linalg.norm(a)
        a /= a_length
        a *= ((1 - a_length) / 2) - MARKER_OFFSET
        origin = left + a
        origin[2] = 0
        return origin

    def get_orientation(self):
        left = self.get_left()
        right = self.get_right()
        x = right - left
        x = x / np.linalg.norm(x)
        z = [0, 0, 1]
        y = np.cross(z,x)
        y = y / np.linalg.norm(y)
        q = quaternion_from_matrix([
            [x[0], y[0], z[0], 0],
            [x[1], y[1], z[1], 0],
            [x[2], y[2], z[2], 0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        return q / np.linalg.norm(q)

    def get_name(self):
        return 'shelf_system_{}'.format(self.id)


class BaseboardDetector(object):
    def __init__(self):
        # TODO use paramserver [low]
        self.shelf_width = 1

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.marker_ns = 'baseboard_marker'

        self.shelves = []
        self.baseboard_detector_topic = '/ros_markers/tf'
        # try:
        #     rospy.wait_for_message('/refills_wrist_camera/image_color', rospy.AnyMsg, timeout=1)
        # except ROSException as e:
        #     rospy.loginfo('camera offline; \'detecting\' shelves anyway')
        #     self.detect_fake_shelves()

        self.left_color = ColorRGBA(1, 1, 0, 1)
        self.right_color = ColorRGBA(1, .5, 0, 1)

    def start_listening(self):
        self.shelves = []
        self.detector_sub = rospy.Subscriber(self.baseboard_detector_topic, TFMessage, self.cb, queue_size=10)

    def stop_listening(self):
        self.detector_sub.unregister()
        # self.publish_as_marker()
        return OrderedDict([x.get_shelf() for x in self.shelves if x.is_complete()])

    def detect_fake_shelves(self, ids):
        y = -0.615
        x = 0.66-MARKER_OFFSET
        if '0' in ids:
            s1 = Shelf(1, [x, y, 0])
            s1.add_measurement(2, [1+x, y, 0])
            self.shelves.append(s1)
        if '1' in ids:
            s2 = Shelf(3, [1+x, y, 0])
            s2.add_measurement(4, [2+x, y, 0])
            self.shelves.append(s2)
        if '2' in ids:
            s3 = Shelf(5, [2+x, y, 0])
            s3.add_measurement(6, [3+x, y, 0])
            self.shelves.append(s3)
        if '3' in ids:
            s4 = Shelf(7, [3+x, y, 0])
            s4.add_measurement(8, [4+x, y, 0])
            self.shelves.append(s4)
        y = -0.615
        x = 0.66-MARKER_OFFSET
        # l = -0.02-MARKER_OFFSET
        # r = +0.07-MARKER_OFFSET
        # if '0' in ids:
        #     s1 = Shelf(20, [0.68+l, -0.6, 0])
        #     s1.add_measurement(21, [1.59+r, -0.58, 0])
        #     self.shelves.append(s1)
        # if '1' in ids:
        #     s2 = Shelf(22, [1.69+l, -0.59, 0])
        #     s2.add_measurement(23, [2.6+r, -0.588, 0])
        #     self.shelves.append(s2)
        # if '2' in ids:
        #     s3 = Shelf(24, [2.69+l, -0.59, 0])
        #     s3.add_measurement(25, [3.588+r, -0.587, 0])
        #     self.shelves.append(s3)
        # if '3' in ids:
        #     s4 = Shelf(26, [3+x, y, 0])
        #     s4.add_measurement(27, [4+x, y, 0])
        #     self.shelves.append(s4)

    def cb(self, data):
        for msg in data.transforms:
            number = int(msg.child_frame_id.split('_')[-1])
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = Point(msg.transform.translation.x,
                                       msg.transform.translation.y,
                                       msg.transform.translation.z)
            pose.pose.orientation.w = 1
            pose = transform_pose(MAP, pose)
            if pose is not None:
                position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                for shelf in self.shelves:
                    if shelf.add_measurement(number, position):
                        break
                else:  # if not break
                    self.shelves.append(Shelf(number, position))

    def publish_as_marker(self):
        # TODO use current frame id
        ma = MarkerArray()
        for i, shelf in enumerate(self.shelves):
            if shelf.is_complete():
                # shelf
                m = Marker()
                m.header.frame_id = shelf.get_name()
                m.ns = self.marker_ns
                m.id = i
                m.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 2, [0, 0, 1]))
                if shelf.id == 0:
                    m.pose.position.y -= 0.07
                m.action = Marker.ADD
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = 'package://iai_shelves/meshes/Shelf_{}.dae'.format(shelf.id)
                m.scale = Vector3(1, 1, 1)
                m.color = ColorRGBA(0, 0, 0, 0)
                m.mesh_use_embedded_materials = True
                ma.markers.append(m)
                # left
                m = Marker()
                m.header.frame_id = MAP
                m.ns = self.marker_ns
                m.id = i + 1000
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position = Point(*shelf.get_left())
                m.pose.position.z = 0.03
                m.pose.orientation = Quaternion(*shelf.get_orientation())
                m.scale = Vector3(.05, .03, .05)
                m.color = self.left_color
                ma.markers.append(m)
                # right
                m = deepcopy(m)
                m.id += 10000
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
    d.detect_fake_shelves(cmd)
    print('baseboard detection test ended')
    print(d.stop_listening())
    rospy.sleep(.5)
