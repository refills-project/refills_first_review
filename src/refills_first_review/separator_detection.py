from sklearn.cluster import DBSCAN

import rospy
import numpy as np

from geometry_msgs.msg import Point, Vector3, PoseStamped
from refills_msgs.msg import SeparatorArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from refills_first_review.tfwrapper import TfWrapper


class SeparatorClustering(object):
    def __init__(self):
        self.tf = TfWrapper()
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.detections = []
        self.map_frame_id = 'map'
        self.separator_detector_topic = '/separator_marker_detector_node/data_out'
        self.separator_maker_color = ColorRGBA(0,0,1,1)

    def start_listening(self, shelf_id, floor_id):
        self.separator_sub = rospy.Subscriber(self.separator_detector_topic, SeparatorArray, self.separator_cb,
                                              queue_size=10)
        self.marker_ns = 'separator_{}_{}'.format(shelf_id, floor_id)

    def stop_listening(self):
        self.separator_sub.unregister()
        separators = self.cluster()
        self.publish_as_marker(separators)
        return separators

    def separator_cb(self, separator_array):
        for separator in separator_array.separators:
            map_pose = self.tf.transformPose(self.map_frame_id, separator.separator_pose)
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
            clusters = DBSCAN(eps=0.01, min_samples=3).fit(data)
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
            m.scale = Vector3(.01,.01,.01)
            m.color = self.separator_maker_color
            ma.markers.append(m)
        self.marker_pub.publish(ma)

if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    s = SeparatorClustering()
    s.start_listening('test', '123')
    print('separator detection test started')
    cmd = raw_input('stop? [enter]')
    print('separator detection test ended')
    print(s.stop_listening())
    rospy.sleep(.5)
