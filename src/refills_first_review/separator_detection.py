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
from refills_first_review.tfwrapper import transform_pose


class SeparatorClustering(object):
    def __init__(self, knowrob):
        self.knowrob = knowrob
        # TODO use paramserver [low]
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.detections = []
        self.map_frame_id = 'map'
        self.separator_maker_color = ColorRGBA(.8, .8, .8, .8)
        self.separator_maker_scale = Vector3(.01, .5, .05)
        self.min_samples = 2
        self.max_dist = 0.015
        self.hanging = False
        self.listen = False
        self.separator_sub = rospy.Subscriber('/separator_marker_detector_node/data_out', SeparatorArray,
                                              self.separator_cb,
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
                p = transform_pose(frame_id, separator.separator_pose)
                if p is not None and self.separator_on_floor(p):
                    self.detections.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])

    def separator_on_floor(self, separator_pose, width_threshold=0.03, height_threshold=0.05):
        """
        :param separator_pose: pose of separator in floor frame
        :type separator_pose: PoseStamped
        :return: bool
        """
        floor_width = self.knowrob.get_floor_width()
        x = separator_pose.pose.position.x
        z = separator_pose.pose.position.z
        return width_threshold <= x and x <= floor_width - width_threshold and \
               -height_threshold <= z and z <= height_threshold

    def cluster(self, visualize=False):
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

            if visualize:
                self.visualize_detections(clusters.labels_, self.pose_list_to_np(separators))
        return separators

    def cluster_to_separator(self, separator_cluster):
        return separator_cluster.mean(axis=0)

    def fake_detection(self):
        if not self.hanging:
            num_fake_separators = 5
            for i in range(num_fake_separators):
                for j in range(self.min_samples + 1):
                    if self.hanging:
                        x = (i + 0.5) / (num_fake_separators - 1)
                    else:
                        x = i / (num_fake_separators - 1)
                    muh = [x,0,0]
                    sigma = 0.00001
                    cov = [[sigma,0,0],
                           [0,sigma,0],
                           [0,0,sigma]]
                    points = np.random.multivariate_normal(muh, cov, 10)
                    for k in range(points.shape[0]):
                        p = points[k]
                        if (self.hanging and i < num_fake_separators - 1) or not self.hanging:
                            self.detections.append(p)

    def hacky(self):
        for i in range(200):
            self.detections.append([0, 0, 0])
            self.detections.append([1, 0, 0])
        for i in range(20):
            self.detections.append([0.01, 0, 0])
            self.detections.append([0.99, 0, 0])
            # self.detections.append([0.02,0,0])
            # self.detections.append([0.98,0,0])
            # self.detections.append([0.03,0,0])
            # self.detections.append([0.97,0,0])

    def pose_list_to_np(self, poses):
        """
        :type poses: list
        :return:
        """
        l = []
        for p in poses: # type: PoseStamped
            l.append([p.pose.position.x,
                      p.pose.position.y,
                      p.pose.position.z])
        return np.array(l)

    def visualize_detections(self, labels, centers):
        import pylab as plt
        from mpl_toolkits.mplot3d import Axes3D

        ulabels = np.unique(labels)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        detections = np.array(self.detections)
        for i, label in enumerate(ulabels):
            if i % 2 == 0:
                color = 'g'
            else:
                color = 'y'
            if label == -1:
                color = 'r'
            ax.scatter(detections[labels==label,0], detections[labels==label,1], detections[labels==label,2], c=color,
                       linewidth=0.0)

        ax.scatter(centers[:,0], centers[:,1], centers[:,2], c='k',
                   marker='x',s=80)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim(0,1)
        ax.set_ylim(-.5,.5)
        ax.set_zlim(-.5,.5)
        plt.show()


if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    kr = KnowRob()
    s = SeparatorClustering(kr)
    shelfs = kr.get_shelves()
    for shelf_id in shelfs:
        try:
            floor = kr.get_floor_ids(shelf_id).keys()[0]
            break
        except:
            pass
    else:
        print('no shelf has floors')
    s.start_listening_separators(floor)
    s.stop_listening()
    # print('separator detection test started')
    # cmd = raw_input('stop? [enter]')
    # print('separator detection test ended')
    # separators = s.stop_listening()
    # print(separators)
    # rospy.sleep(.5)
