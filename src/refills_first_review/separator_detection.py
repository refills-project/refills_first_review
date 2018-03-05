from __future__ import division, print_function
from sklearn.cluster import DBSCAN

import rospy
import numpy as np

from geometry_msgs.msg import Point, Vector3, PoseStamped
from refills_msgs.msg import SeparatorArray
from rospy import ROSException
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from refills_first_review.tfwrapper import TfWrapper


class SeparatorClustering(object):
    def __init__(self):
        # TODO use paramserver [low]
        self.tf = TfWrapper()
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.detections = []
        self.map_frame_id = 'map'
        self.separator_detector_topic = '/separator_marker_detector_node/data_out'
        self.separator_maker_color = ColorRGBA(.8, .8, .8, .8)
        # TODO change marker shape [low]
        # TODO return correct number of separators in fake mode [low]
        self.separator_maker_scale = Vector3(.01, .5, .05)
        self.min_samples = 2
        self.max_dist = 0.02

    def start_listening(self, shelf_id, floor_id):
        #TODO zick zack hack
        self.current_floor_id = floor_id

        self.detections = []
        self.separator_sub = rospy.Subscriber(self.separator_detector_topic, SeparatorArray, self.separator_cb,
                                              queue_size=10)
        self.marker_ns = 'separator_{}_{}'.format(shelf_id, floor_id)

    def stop_listening(self):
        self.separator_sub.unregister()
        try:
            rospy.wait_for_message('/refills_wrist_camera/image_color', rospy.AnyMsg, timeout=1)
        except ROSException as e:
            rospy.loginfo('camera offline; \'detecting\' separators anyway')
            self.fake_detection()
        separators = self.cluster()
        self.publish_as_marker(separators)
        return separators

    def separator_cb(self, separator_array):
        for separator in separator_array.separators:
            map_pose = self.tf.transform_pose(self.map_frame_id, separator.separator_pose)
            if map_pose is not None:
                position = [map_pose.pose.position.x,
                            map_pose.pose.position.y,
                            map_pose.pose.position.z]
                self.detections.append(position)

    def cluster(self):
        data = np.array(self.detections)
        separators = []
        if len(data) == 0:
            rospy.logwarn('no separators detected')
        else:
            clusters = DBSCAN(eps=self.max_dist, min_samples=self.min_samples).fit(data)
            labels = np.unique(clusters.labels_)
            rospy.loginfo('detected {} separators'.format(len(labels)))
            for i, label in enumerate(labels):
                if label != -1:
                    separator = PoseStamped()
                    separator.header.frame_id = self.map_frame_id
                    separator.header.stamp = rospy.get_rostime()
                    separator.pose.position = Point(*self.cluster_to_separator(data[clusters.labels_ == label]))
                    separator.pose.orientation.w = 1.0
                    separators.append(separator)
        return separators

    def cluster_to_separator(self, separator_cluster):
        return separator_cluster.mean(axis=0)

    def publish_as_marker(self, separators):
        ma = MarkerArray()
        for i, separator in enumerate(separators):
            m = Marker()
            m.header = separator.header
            m.ns = self.marker_ns
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = separator.pose
            m.pose.position.y -= self.separator_maker_scale.y / 2
            m.scale = self.separator_maker_scale
            m.color = self.separator_maker_color
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def fake_detection(self):
        num_fake_separators = 6
        for i in range(num_fake_separators):
            for j in range(4):
                p = PoseStamped()
                p.header.frame_id = 'camera_link'
                #TODO zick zack hack
                if self.current_floor_id % 2 == 0:
                    p.pose.position = Point(-i * 1 / (num_fake_separators - 1), 0, 0.25)
                else:
                    p.pose.position = Point(i * 1 / (num_fake_separators - 1), 0, 0.25)
                p = self.tf.transform_pose(self.map_frame_id, p)
                self.detections.append([p.pose.position.x,
                                        p.pose.position.y,
                                        p.pose.position.z, ])


if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    s = SeparatorClustering()
    s.start_listening('test', '123')
    print('separator detection test started')
    cmd = raw_input('stop? [enter]')
    print('separator detection test ended')
    print(s.stop_listening())
    rospy.sleep(.5)
