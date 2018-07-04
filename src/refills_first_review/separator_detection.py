from __future__ import division, print_function

from sklearn.cluster import DBSCAN

import rospy
import numpy as np

from copy import deepcopy
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from refills_msgs.msg import SeparatorArray
from rospy import ROSException
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_about_axis
from visualization_msgs.msg import Marker, MarkerArray

from refills_first_review.knowrob_wrapper import KnowRob
from refills_first_review.tfwrapper import TfWrapper


class SeparatorClustering(object):
    def __init__(self, knowrob):
        self.knowrob = knowrob
        # TODO use paramserver [low]
        self.tf = TfWrapper(6)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.detections = []
        self.map_frame_id = 'map'
        self.separator_maker_color = ColorRGBA(.8, .8, .8, .8)
        self.separator_maker_scale = Vector3(.01, .5, .05)
        self.min_samples = 1
        self.max_dist = 0.025
        self.hanging = False
        self.listen = False
        self.separator_sub = rospy.Subscriber('/separator_marker_detector_node/data_out', SeparatorArray, self.separator_cb,
                                              queue_size=10)

    def start_listening_separators(self, floor_id, topic='/separator_marker_detector_node/data_out'):
        self.hanging = False
        # self.topic = topic
        self.current_floor_id = floor_id
        self.listen = True
        self.detections = []
        self.marker_ns = 'separator_{}'.format(floor_id)

    def start_listening_mounting_bars(self, floor_id):
        self.start_listening_separators(floor_id, topic='/muh')
        self.hanging = True

    def stop_listening(self):
        self.listen = False
        try:
            rospy.wait_for_message('/refills_wrist_camera/image_color', rospy.AnyMsg, timeout=5)
        except ROSException as e:
            rospy.loginfo('camera offline; \'detecting\' separators anyway')
            self.fake_detection()
        separators = self.cluster()
        return separators

    def separator_cb(self, separator_array):
        if self.listen:
            frame_id = self.knowrob.get_perceived_frame_id(self.current_floor_id)
            for separator in separator_array.separators:
                p = self.tf.transform_pose(frame_id, separator.separator_pose)
                if p is not None and 0.04 <= p.pose.position.x and p.pose.position.x <= 0.96:
                    self.detections.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])

    def cluster(self):
        if not self.hanging:
            self.hacky()
        data = np.array(self.detections)
        separators = []
        old_frame_id = self.knowrob.get_perceived_frame_id(self.current_floor_id)
        if len(data) == 0:
            rospy.logwarn('no separators detected')
        else:
            clusters = DBSCAN(eps=self.max_dist, min_samples=self.min_samples).fit(data)
            labels = np.unique(clusters.labels_)
            rospy.loginfo('detected {} separators'.format(len(labels)))
            for i, label in enumerate(labels):
                if label != -1:
                    separator = PoseStamped()
                    separator.header.frame_id = old_frame_id
                    separator.pose.position = Point(*self.cluster_to_separator(data[clusters.labels_ == label]))
                    separator.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 2, [0, 0, 1]))
                    if 0.0 <= separator.pose.position.x and separator.pose.position.x <= 1:
                        separators.append(separator)

        return separators

    def cluster_to_separator(self, separator_cluster):
        return separator_cluster.mean(axis=0)

    def fake_detection(self):
        if not self.hanging:
            num_fake_separators = 15
            frame_id = self.knowrob.get_perceived_frame_id(self.current_floor_id)
            for i in range(num_fake_separators):
                for j in range(self.min_samples + 1):
                    p = PoseStamped()
                    p.header.frame_id = frame_id
                    if self.hanging:
                        p.pose.position.x = (i+0.5) / (num_fake_separators-1)
                    else:
                        p.pose.position.x = i / (num_fake_separators - 1)
                    p.pose.position.y = 0
                    p.pose.orientation = Quaternion(0, 0, 0, 1)
                    if (self.hanging and i < num_fake_separators - 1) or not self.hanging:
                        self.detections.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])

    def hacky(self):
        for i in range(200):
            self.detections.append([0,0,0])
            self.detections.append([1,0,0])
        for i in range(20):
            self.detections.append([0.01,0,0])
            self.detections.append([0.99,0,0])
            # self.detections.append([0.02,0,0])
            # self.detections.append([0.98,0,0])
            # self.detections.append([0.03,0,0])
            # self.detections.append([0.97,0,0])


if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    s = SeparatorClustering(KnowRob())
    s.start_listening_separators('http://knowrob.org/kb/dm-market.owl#DMShelfLayer4TilesFront_UNVRGYSE')
    print('separator detection test started')
    cmd = raw_input('stop? [enter]')
    print('separator detection test ended')
    separators = s.stop_listening()
    print(separators)
    rospy.sleep(.5)
