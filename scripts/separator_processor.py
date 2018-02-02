from sklearn.cluster import DBSCAN

import rospy
import numpy as np

from geometry_msgs.msg import Point, Vector3
from refills_msgs.msg import SeparatorArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from tf2_geometry_msgs import do_transform_pose
from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


class SeparatorCluster(object):
    def __init__(self):
        self.tfBuffer = Buffer(rospy.Duration(2))
        self.tf_listener = TransformListener(self.tfBuffer)
        rospy.sleep(0.1)
        self.separator_sub = rospy.Subscriber('/separator_marker_detector_node/data_out', SeparatorArray,
                                              self.separator_cb, queue_size=10)
        self.cluster_service = rospy.Service('~cluster', Trigger, self.cluster_cb)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.detections = []

    def transformPose(self, target_frame, pose):
        try:
            transform = self.tfBuffer.lookup_transform(target_frame,
                                                       pose.header.frame_id,  # source frame
                                                       pose.header.stamp,
                                                       rospy.Duration(1.0))
            new_pose = do_transform_pose(pose, transform)
            return new_pose
        except ExtrapolationException as e:
            rospy.logwarn(e)

    def separator_cb(self, separator_array):
        for separator in separator_array.separators:
            map_pose = self.transformPose('map', separator.separator_pose)
            if map_pose is not None:
                position = [map_pose.pose.position.x,
                            map_pose.pose.position.y,
                            map_pose.pose.position.z]
                self.detections.append(position)

    def cluster_cb(self, _):
        data = np.array(self.detections)
        if len(data) == 0:
            rospy.logwarn('no separators detected')
        clusters = DBSCAN(eps=0.01, min_samples=3).fit(data)
        labels = np.unique(clusters.labels_)
        rospy.loginfo('detected {} separators'.format(len(labels)))
        for i, label in enumerate(labels):
            if label != -1:
                p = data[clusters.labels_==label].mean(axis=0)
                self.marker_pub.publish(self.create_marker(p, i))

        return TriggerResponse()

    def create_marker(self, p, id):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'separator'
        m.id = id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position = Point(*p)
        m.pose.orientation.w = 1
        m.scale = Vector3(0.01,0.01,0.01)
        m.color = ColorRGBA(1,0,0,1)
        return m

if __name__ == '__main__':
    rospy.init_node('separator_processor')
    muh = SeparatorCluster()
    rospy.spin()
