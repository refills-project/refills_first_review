from __future__ import division

import json
from random import choice, sample
from simplejson import OrderedDict

import rospy
import numpy as np

from copy import deepcopy

from collections import defaultdict
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion, Pose
from refills_msgs.msg import Barcode
from rospy import ROSException
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_matrix, quaternion_from_euler
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from rospkg import RosPack
from refills_first_review.tfwrapper import TfWrapper

MAP = 'map'


class BarcodeDetector(object):
    def __init__(self, knowrob):
        self.knowrob = knowrob
        self.load_barcode_to_mesh_map()
        # TODO use paramserver [low]
        self.shelf_width = 1

        self.tf = TfWrapper(4)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.marker_object_ns = 'barcode_object'
        self.marker_text_ns = 'barcode_text'

        self.detector_topic = 'barcode/pose'
        self.refills_models_path = 'package://refills_models/'

        self.object_color = ColorRGBA(0, 0, 0, 1)
        self.text_color = ColorRGBA(1, 1, 1, 1)
        self.object_scale = Vector3(.05, .05, .05)
        self.text_scale = Vector3(0, 0, .05)
        self.listen = False
        self.sub = rospy.Subscriber(self.detector_topic, Barcode, self.cb, queue_size=100)

    def load_barcode_to_mesh_map(self):
        self.barcode_to_mesh = json.load(open(RosPack().get_path('refills_first_review')+'/data/barcode_to_mesh.json'))

    def start_listening(self, shelf_id, floor_id):
        self.shelf_id = shelf_id
        self.floor_id = floor_id
        self.barcodes = defaultdict(list)
        self.listen = True
        # self.sub = rospy.Subscriber(self.detector_topic, Barcode, self.cb, queue_size=100)

    def stop_listening(self):
        # self.sub.unregister()
        self.listen = False
        try:
            rospy.wait_for_message('/refills_wrist_camera/image_color', rospy.AnyMsg, timeout=5)
        except ROSException as e:
            rospy.loginfo('camera offline; \'detecting\' barcodes anyway')
            self.detect_fake_barcodes()
        self.cluster()
        self.publish_as_marker()
        rospy.loginfo('detected {} barcodes'.format(len(self.barcodes)))
        return self.barcodes

    def detect_fake_barcodes(self):
        num_of_barcodes = 14
        frame_id = self.knowrob.get_perceived_frame_id(self.floor_id)
        barcodes = sample(self.barcode_to_mesh.keys(), num_of_barcodes)
        for i in range(num_of_barcodes):
            barcode = barcodes[i]
            p = PoseStamped()
            p.header.frame_id = frame_id
            p.pose.position.x = (i + .5) / (num_of_barcodes)
            p.pose.position.y = 0
            p.pose.orientation = Quaternion(0, 0, 0, 1)
            self.barcodes[barcode].append(p)

    def cluster(self):
        frame_id = self.knowrob.get_perceived_frame_id(self.floor_id)
        for barcode, poses in self.barcodes.items():
            positions = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses]
            position = np.mean(positions, axis=0)
            p = PoseStamped()
            p.header.frame_id = frame_id
            p.pose.position = Point(*position)
            p.pose.orientation.w = 1
            self.barcodes[barcode] = p

    def cb(self, data):
        if self.listen:
            p = self.tf.transform_pose(MAP, data.barcode_pose)
            if p is not None:
                p.header.stamp = rospy.Time()
                p = self.tf.transform_pose(self.knowrob.get_perceived_frame_id(self.floor_id), p)
                if p.pose.position.x > 0.0 and p.pose.position.x < 1.0:
                    self.barcodes[data.barcode[1:-1]].append(p)

    def publish_as_marker(self):
        ma = MarkerArray()
        frame_id = self.knowrob.get_perceived_frame_id(self.floor_id)
        for i, (barcode, pose) in enumerate(self.barcodes.items()):
            # object
            m = Marker()
            m.header.frame_id = frame_id
            m.ns = self.marker_object_ns
            m.id = int(barcode)
            m.action = Marker.ADD
            m.pose = deepcopy(pose.pose)
            try:
                mesh_path = self.barcode_to_mesh[str(barcode)]
            except KeyError as e:
                mesh_path = ''
            if mesh_path == '':
                m.type = Marker.CUBE
                m.pose.orientation = Quaternion(0, 0, 0, 1)
                m.scale = self.object_scale
                m.color = self.object_color
            else:
                m.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, -np.pi / 2))
                m.pose.position.y = -0.03
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = self.refills_models_path + mesh_path
                m.scale = Vector3(1, 1, 1)
                m.color = ColorRGBA(0, 0, 0, 0)
                m.mesh_use_embedded_materials = True
            # if mesh_path != '':
            #     ma.markers.append(m)

            # text
            m = Marker()
            m.header.frame_id = frame_id
            m.ns = self.marker_text_ns
            m.id = int(barcode)
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.text = barcode
            m.scale = self.text_scale
            m.color = self.text_color
            m.pose = deepcopy(pose.pose)
            m.pose.position.z += 0.07
            ma.markers.append(m)
        if len(ma.markers) > 0:
            self.marker_pub.publish(ma)


if __name__ == '__main__':
    rospy.init_node('baseboard_detection_test')
    d = BarcodeDetector(True)
    d.start_listening('shelf_system_1', 2)
    print('barcode detection test started')
    cmd = raw_input('stop? [enter]')
    print('barcode detection test ended')
    print(d.stop_listening())
    rospy.sleep(.5)
