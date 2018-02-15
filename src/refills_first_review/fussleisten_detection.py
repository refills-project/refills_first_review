from __future__ import division

from simplejson import OrderedDict
from sklearn.cluster import DBSCAN

import rospy
import numpy as np

from copy import deepcopy
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion, Pose
from refills_msgs.msg import SeparatorArray
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray

from refills_first_review.tfwrapper import TfWrapper


class FussleistenDetection(object):
    def __init__(self):
        self.shelf_width = 1
        self.map_frame_id = 'map'

        self.tf = TfWrapper()
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.marker_ns = 'fussleisten_marker'

        # TODO make empty when using real shit
        self.left_markers = np.array([
            [-0.05, 0.515, 0],
            [-1.05, 0.515, 0],
            [-2.05, 0.515, 0],
            [-3.05, 0.515, 0],
        ])
        self.right_markers = np.array([
            [-0.95, 0.515, 0],
            [-1.95, 0.515, 0],
            [-2.95, 0.515, 0],
            [-3.95, 0.515, 0],
        ])
        self.separator_detector_topic = '/muh'
        self.left_color = ColorRGBA(1, 1, 0, 1)
        self.right_color = ColorRGBA(1, .5, 0, 1)

    def start_listening(self):
        # self.separator_sub = rospy.Subscriber(self.separator_detector_topic, SeparatorArray, self.separator_cb,
        #                                       queue_size=10)
        # self.marker_ns = 'separator_{}_{}'.format(shelf_id, floor_id)
        pass

    def stop_listening(self):
        # self.separator_sub.unregister()
        # separators = self.cluster()
        self.publish_as_marker(self.left_markers, self.right_markers)
        self.shelves = []
        for i, (left, right) in enumerate(zip(self.left_markers, self.right_markers)):
            shelf_name = 'shelf{}'.format(i)
            self.shelves.append((shelf_name, PoseStamped(Header(0, rospy.Time(), self.map_frame_id),
                                                         Pose(Point(*self.calc_shelf_origin(left, right)),
                                                              Quaternion(*self.get_orientation(left, right))))))
        return OrderedDict(self.shelves)

    def calc_shelf_origin(self, left, right):
        a = left - right
        a_length = np.linalg.norm(a)
        a /= a_length
        a *= (1-a_length)/2
        return left+a

    def cb(self, data):
        pass

    def cluster(self):
        pass

    def get_orientation(self, left, right):
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

    def publish_as_marker(self, left_marker, right_marker):
        ma = MarkerArray()
        for i, (left, right) in enumerate(zip(left_marker, right_marker)):
            # left
            m = Marker()
            m.header.frame_id = self.map_frame_id
            m.ns = self.marker_ns
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(*left)
            m.pose.position.z = 0.03
            m.pose.orientation = Quaternion(*self.get_orientation(left, right))
            m.scale = Vector3(.02, .05, .05)
            m.color = self.left_color
            ma.markers.append(m)
            # right
            m = deepcopy(m)
            m.id += 900000
            m.pose.position = Point(*right)
            m.pose.position.z = 0.03
            m.color = self.right_color
            ma.markers.append(m)
        self.marker_pub.publish(ma)
