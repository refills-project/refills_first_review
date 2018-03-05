from __future__ import division

import json
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

from refills_first_review.tfwrapper import TfWrapper

MAP = 'map'


class BarcodeDetector(object):
    def __init__(self):
        self.load_barcode_to_mesh_map()
        # TODO use paramserver [low]
        self.shelf_width = 1

        self.tf = TfWrapper()
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.marker_object_ns = 'barcode_object'
        self.marker_text_ns = 'barcode_text'

        self.detector_topic = 'barcode/pose'
        self.refills_models_path = 'package://refills_models/'

        self.object_color = ColorRGBA(0, 0, 0, 1)
        self.text_color = ColorRGBA(1, 1, 1, 1)
        self.object_scale = Vector3(.05, .05, .05)
        self.text_scale = Vector3(0, 0, .05)


    def load_barcode_to_mesh_map(self):
        try:
            self.barcode_to_mesh = json.load(open('../data/barcode_to_mesh.json'))
        except IOError:
            self.barcode_to_mesh = json.load(open('../../data/barcode_to_mesh.json'))

        self.offline_i = 0

    def start_listening(self, shelf_id, floor_id):
        self.shelf_id = shelf_id
        self.floor_id = floor_id
        self.barcodes = defaultdict(list)
        self.sub = rospy.Subscriber(self.detector_topic, Barcode, self.cb, queue_size=100)

    def stop_listening(self):
        self.sub.unregister()
        try:
            rospy.wait_for_message('/refills_wrist_camera/image_color', rospy.AnyMsg, timeout=1)
        except ROSException as e:
            rospy.loginfo('camera offline; \'detecting\' barcodes anyway')
            self.detect_fake_barcodes()
        self.cluster()
        self.publish_as_marker()
        rospy.loginfo('detected {} barcodes'.format(len(self.barcodes)))
        return self.barcodes

    def detect_fake_barcodes(self):
        if self.shelf_id in self.barcode_to_mesh:
            barcodes_len = len(self.barcode_to_mesh[self.shelf_id][str(self.floor_id)])
            for i, barcode in enumerate(self.barcode_to_mesh[self.shelf_id][str(self.floor_id)]):

                p = PoseStamped()
                p.header.frame_id = 'camera_link'
                # TODO zick zack hack
                if self.floor_id % 2 == 0:
                    p.pose.position = Point(-(i + .5) * 1 / (barcodes_len), 0.025, 0.25)
                else:
                    p.pose.position = Point((i + .5) * 1 / (barcodes_len), 0.025, 0.25)

                p = self.tf.transform_pose(MAP, p)
                self.barcodes[barcode].append([p.pose.position.x,
                                               p.pose.position.y,
                                               p.pose.position.z, ])

    def cluster(self):
        for barcode, positions in self.barcodes.items():
            self.barcodes[barcode] = np.mean(positions, axis=0)

    def cb(self, data):
        p = self.tf.transform_pose(MAP, data.barcode_pose).pose.position
        self.barcodes[data.barcode].append([p.x, p.y, p.z])

    def publish_as_marker(self):
        ma = MarkerArray()
        for i, (barcode, position) in enumerate(self.barcodes.items()):
            # object
            m = Marker()
            m.header.frame_id = MAP
            m.ns = self.marker_object_ns
            m.id = int(barcode)
            m.action = Marker.ADD
            m.pose.position = Point(*position)
            try:
                mesh_path = self.barcode_to_mesh[self.shelf_id][str(self.floor_id)][str(barcode)[1:-1]]
            except KeyError as e:
                mesh_path = ''
            if mesh_path == '':
                m.type = Marker.CUBE
                m.pose.orientation = Quaternion(0, 0, 0, 1)
                m.scale = self.object_scale
                m.color = self.object_color
            else:
                m.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.pi / 2))
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = self.refills_models_path + mesh_path
                m.scale = Vector3(1, 1, 1)
                m.color = ColorRGBA(0, 0, 0, 0)
                m.mesh_use_embedded_materials = True
            ma.markers.append(m)

            # text
            m = Marker()
            m.header.frame_id = MAP
            m.ns = self.marker_text_ns
            m.id = int(barcode)
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.text = barcode
            m.scale = self.text_scale
            m.color = self.text_color
            m.pose.position = Point(*position)
            m.pose.position.z += 0.07
            ma.markers.append(m)
        if len(ma.markers) > 0:
            self.marker_pub.publish(ma)


if __name__ == '__main__':
    rospy.init_node('baseboard_detection_test')
    d = BarcodeDetector()
    d.start_listening('shelf0',0)
    print('barcode detection test started')
    cmd = raw_input('stop? [enter]')
    print('barcode detection test ended')
    print(d.stop_listening())
    rospy.sleep(.5)
