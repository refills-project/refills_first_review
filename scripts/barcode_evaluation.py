#!/usr/bin/env python
import json
import numpy as np
from time import localtime

import rospy
from collections import defaultdict
from geometry_msgs.msg import Vector3
from refills_msgs.msg import Barcode
from rospy_message_converter.json_message_converter import convert_ros_message_to_json
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


class Data(object):
    def __init__(self):
        self.detected_barcodes = defaultdict(list)

    def add_barcode(self, data):
        self.detected_barcodes[data.barcode].append(convert_ros_message_to_dictionary(data.barcode_pose))

    def save_to_file(self):
        rospy.loginfo('saved scanned barcodes')
        t = localtime()
        date = '-'.join([str(x) for x in [t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec]])
        json.dump(self.detected_barcodes, open('../data/scanning_data_{}.json'.format(date), 'w'),
                  sort_keys=True,
                  indent=2,
                  separators=(',', ': '))


class Eval(object):
    def __init__(self):
        self.tfBuffer = Buffer(rospy.Duration(10))
        self.tf_listener = TransformListener(self.tfBuffer)
        self.barcode_pose_sub = rospy.Subscriber('barcode/pose', Barcode, self.barcode_sub, queue_size=100)
        self.done = rospy.Service('barcode_evaluation/done', Trigger, self.done_cb)
        self.marker_pub = rospy.Publisher('wrong_codes', Marker, queue_size=10)
        self.barcodes = {}
        self.tp = {}
        self.fp = {}
        self.data = Data()
        self.load_ground_truth()
        rospy.loginfo('rdy to evaluate barcode detection')
        rospy.sleep(1)

    def make_marker(self, barcode_msg):
        ean = barcode_msg.barcode
        m = Marker()
        m.header = barcode_msg.barcode_pose.header
        m.pose = barcode_msg.barcode_pose.pose
        m.action = Marker.ADD
        m.type = Marker.CUBE
        m.scale = Vector3(.06, .06, .06)
        m.color = ColorRGBA(1, 0, 0, 1.0)

        text = Marker()
        text.header = barcode_msg.barcode_pose.header
        text.action = Marker.ADD
        text.type = Marker.TEXT_VIEW_FACING
        text.text = ean
        text.scale = Vector3(0, 0, .06)
        text.color = ColorRGBA(1, 0, 0, 1)
        text.pose.position.x = barcode_msg.barcode_pose.pose.position.x
        text.pose.position.y = barcode_msg.barcode_pose.pose.position.y
        text.pose.position.z = barcode_msg.barcode_pose.pose.position.z + 0.05

        m.text = ean
        m.ns = ean
        text.ns = ean
        m.id = 2
        self.marker_pub.publish(m)
        self.marker_pub.publish(text)

    def transformPose(self, target_frame, pose):
        transform = self.tfBuffer.lookup_transform(target_frame,
                                                   pose.header.frame_id,  # source frame
                                                   pose.header.stamp,  # get the tf at first available time
                                                   rospy.Duration(1.0))
        new_pose = do_transform_pose(pose, transform)
        return new_pose

    def done_cb(self, req):
        self.data.save_to_file()
        resp = TriggerResponse()
        resp.success = True
        self.final_results()
        return resp

    def load_ground_truth(self):
        # data = json.load(open('../data/iai_shelfs_barcodes.json'))
        data = json.load(open('../data/ground_truth.json'))
        for shelf in data.values():
            for row in shelf:
                for barcode in row['barcode']:
                    self.barcodes[str(barcode['code'])[:-1]] = np.array([-barcode['pose']['position']['x'],
                                                                         0.515 - barcode['pose']['position']['y'],
                                                                         0.16 + barcode['pose']['position']['z']])

    def final_results(self):
        tp = len(self.tp)
        fp = len(self.fp)
        fn = len(self.barcodes) - tp
        rospy.loginfo('tp: {}, fp: {}, fn: {}'.format(tp, fp, fn))

        self.tp = {}
        self.fp = {}

    def barcode_sub(self, data):
        inner_barcode = data.barcode[1:-1]
        transformed_pose = self.tfBuffer.lookup_transform('map', data.barcode, rospy.Time(),
                                                          timeout=rospy.Duration(4))
        data.barcode_pose = self.transformPose('map', data.barcode_pose)
        p_map = transformed_pose.transform.translation
        p = np.array([p_map.x,
                      p_map.y,
                      p_map.z])
        self.data.add_barcode(data)
        try:
            p2 = self.barcodes[inner_barcode]

            if inner_barcode not in self.tp:
                self.tp[inner_barcode] = p
                print('ground truth {}; predicted {}'.format(p2, p))
                rospy.loginfo('detected {} position diff: {}'.format(inner_barcode, np.linalg.norm(p - p2)))
                print('-----')
        except KeyError as e:
            if inner_barcode not in self.fp:
                rospy.logwarn('unknown barcode {} at {} {}'.format(inner_barcode, p, data))
                self.fp[inner_barcode] = p
                self.make_marker(data)


if __name__ == '__main__':
    rospy.init_node('barcode_evaluator')
    muh = Eval()
    rospy.spin()
