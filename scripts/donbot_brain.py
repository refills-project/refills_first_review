#!/usr/bin/env python

from __future__ import print_function, division

import traceback
import numpy as np
from time import time
import datetime
import os
from subprocess import call

import rospy
from rospkg import RosPack
from actionlib import SimpleActionServer
from copy import deepcopy
from geometry_msgs.msg import QuaternionStamped, Quaternion, PointStamped, Point, PoseStamped, Pose
from multiprocessing import TimeoutError

from refills_msgs.msg import ScanningFeedback
from refills_msgs.msg._ScanningAction import ScanningAction
from refills_msgs.msg._ScanningGoal import ScanningGoal
from std_msgs.msg import Header

from refills_first_review.knowrob_wrapper import KnowRob
from refills_first_review.move_arm import GiskardWrapper
from refills_first_review.move_base import MoveBase
from refills_first_review.robosherlock_wrapper import RoboSherlock

# base
# shelf id
from refills_first_review.tfwrapper import TfWrapper

#in shelf_id
FLOOR_SCANNING_OFFSET = {'x': -0.18,
                         'y': -0.96,
                         'z': np.pi}

# shelf id
FLOOR_DETECTION_OFFSET = {'x': 0.5,
                          'y': -1.3,
                          'z': np.pi}

# arm
# trans in camera_link, rot in base_footprint
# COUNTING_OFFSET = {'trans': [0.0, -0.15, -0.1],
#                    'rot': [-0.000, 0.805, -0.593, -0.000]
#                    }
# in floor_id
COUNTING_OFFSET = PoseStamped(Header(0, rospy.Time(), ''),
                              Pose(Point(0.097, -0.32, 0.16),
                                   Quaternion(-0.748, 0.000, -0.000, 0.664)))
COUNTING_OFFSET2 = -0.18

# in base_footprint
FLOOR_SCAN_POSE_BOTTOM = {'trans': [-.15, -.69, 0.177],
                          'rot': [0, 0.858, -0.514, 0]}
# in base_footprint
# FLOOR_SCAN_POSE_REST = {'trans': [-.15, -.645, -0.0],
#                         'rot': [-0.111, -0.697, 0.699, 0.111]}
FLOOR_SCAN_POSE_REST = {'trans': [-.15, -.647, 0.0],
                        'rot': [0, 0.7071, -0.7071, 0]}
SHELF_BASEBOARD = PoseStamped(Header(0, rospy.Time(), 'base_footprint'),
                              Pose(Point(-0.137, -0.68, 0.223),
                                   Quaternion(-0.000, 0.841, -0.541, 0.000)))

ACTION_NAME = 'scanning_action'


class CRAM(object):
    def __init__(self):
        # TODO use paramserver [low]
        # TODO live logging [high]
        self._as = SimpleActionServer(ACTION_NAME, ScanningAction, execute_cb=self.action_cb, auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        self.knowrob = KnowRob()
        self.robosherlock = RoboSherlock(self.knowrob)
        self.move_base = MoveBase(enabled=True, knowrob=self.knowrob)
        self.move_arm = GiskardWrapper(enabled=True, knowrob=self.knowrob)
        self.map_frame_id = rospy.get_param('~/map', 'map')
        self.tf = TfWrapper()

    def start(self):
        self.knowrob.start_everything()
        self._as.start()
        rospy.loginfo('waiting for scanning action goal')

    def preempt_cb(self):
        self.STOP()

    def action_cb(self, goal):
        if goal.type != ScanningGoal.COMPLETE_SCAN:
            rospy.logerr('only complete scans are supported')
            self._as.set_aborted('only complete scans are supported')
        try:
            # try:
            #     self.mongo_whipe()
            # except OSError as exc:  # Python >2.5
            #     rospy.logwarn('failed to whipe mongo, IO error')
            
            self.move_arm.drive_pose()
            # TODO scan shelf system
            # self.knowrob.start_shelf_system_mapping(self.sys)
            for i, shelf_id in enumerate(goal.loc_id):
                rospy.loginfo('scanning {}'.format(shelf_id))
                self.scan_shelf(shelf_id)
                # TODO feedback
            self.knowrob.finish_action()
            
            try:
                # data_path = '{}/data/'.format(RosPack().get_path('refills_first_review'))
                data_path = '/tmp'
                episode_name = str(datetime.date.today()) + '_' + str(time())
                episode_dir = data_path+episode_name
                os.makedirs(episode_dir)
                self.knowrob.save_beliefstate(episode_dir+'/beliefstate.owl')
                self.knowrob.save_action_graph(episode_dir+'/actions.owl')
                # self.mongo_save(episode_dir)
            except OSError as exc:  # Python >2.5
                rospy.logwarn('failed to export logs, IO error')
            #self.knowrob.save_beliefstate()
            #self.knowrob.save_action_graph()
            
            self._as.set_succeeded()
        except Exception as e:
            traceback.print_exc()
            rospy.loginfo('preempted')

    def mongo_save(self,out_dir):
        call(['mongodump',
              '--db', 'REFILLS_0',
              '--collection', 'tf',
              '--out', out_dir])
    def mongo_whipe(self):
        call(['mongo',
              'REFILLS_0',
              '--eval', "'db.dropDatabase()'"])

    def detect_baseboards(self):
        rospy.loginfo('shelf baseboard detection requires manuel mode')
        rospy.loginfo('move to free space plx')
        cmd = raw_input('done? [y]')
        while raw_input('add new shelf system? [y]') == 'y':
            shelf_system_id = self.knowrob.add_shelf_system()
            rospy.loginfo('added shelf system {}'.format(shelf_system_id))
            rospy.loginfo('moving arm to baseboard scanning pose')
            self.move_arm.pre_baseboard_pose()
            self.move_arm.set_and_send_cartesian_goal(SHELF_BASEBOARD)
            rospy.loginfo('scan all shelf baseboard plx')
            self.robosherlock.start_baseboard_detection()

            cmd = raw_input('finished scanning shelf system? [y]')
            if cmd == '1337':
                rospy.logwarn('skipping baseboard detection')
                self.robosherlock.baseboard_detection.detect_fake_shelves('0123')

            shelves = self.robosherlock.stop_baseboard_detection()
            self.knowrob.add_shelves(shelf_system_id, shelves)
        rospy.loginfo('MAKE SURE NOTHING IS CLOSE!!!!11elf')
        cmd = raw_input('rdy? [y]')
        if cmd == 'y':
            self.move_arm.drive_pose()
        else:
            raise UserWarning('learn to press \'y\' n00b')

    def scan_shelf(self, shelf_id):
        feedback = ScanningFeedback()
        feedback.current_loc_id = shelf_id
        feedback.progress = 0
        self._as.publish_feedback(feedback)

        self.knowrob.start_shelf_frame_mapping(shelf_id)

        self.knowrob.start_move_to_shelf_frame(shelf_id)
        self.move_base.move_absolute_xyz(self.knowrob.get_perceived_frame_id(shelf_id),
                                         FLOOR_DETECTION_OFFSET['x'],
                                         FLOOR_DETECTION_OFFSET['y'],
                                         FLOOR_DETECTION_OFFSET['z'])
        self.knowrob.finish_action()

        floor_ids = self.detect_shelf_floors(shelf_id)

        num_of_steps = len(floor_ids) + 1

        feedback.progress = 1 / num_of_steps
        self._as.publish_feedback(feedback)

        for i, floor_id in enumerate(floor_ids):
            self.shelf_layer_mapping(shelf_id, floor_id)
            feedback.progress = (i + 1) / num_of_steps
            self._as.publish_feedback(feedback)
        self.move_arm.drive_pose()
        self.knowrob.finish_action()

    def detect_shelf_floors(self, shelf_id):
        self.knowrob.start_finding_shelf_layer()
        self.robosherlock.start_floor_detection(shelf_id)
        self.knowrob.start_looking_at_location(shelf_id)
        self.move_arm.floor_detection_pose()
        if self.robosherlock.robosherlock:
            rospy.sleep(10)
        self.knowrob.finish_action()
        self.knowrob.start_looking_at_location(shelf_id)
        self.move_arm.floor_detection_pose2()
        if self.robosherlock.robosherlock:
            rospy.sleep(10)
        self.knowrob.finish_action()
        floor_heights = self.robosherlock.stop_floor_detection(shelf_id)
        self.knowrob.add_shelf_floors(shelf_id, floor_heights)
        floor_ids = self.knowrob.get_floor_ids(shelf_id)
        self.knowrob.start_shelf_layer_perception(floor_ids)
        self.knowrob.finish_action()
        self.knowrob.finish_action()
        return floor_ids

    def shelf_layer_mapping(self, shelf_id, floor_id):
        self.knowrob.start_shelf_layer_mapping(floor_id)
        self.scan_floor(shelf_id, floor_id)
        # if not self.knowrob.is_hanging_foor(shelf_floor_id):
        self.count_floor(shelf_id, floor_id)
        self.knowrob.finish_action()

    def scan_floor(self, shelf_id, floor_id):
        rospy.loginfo('scanning floor {}'.format(floor_id))
        self.knowrob.start_finding_shelf_layer_parts()

        self.knowrob.start_move_to_shelf_layer()

        self.set_floor_scan_pose(shelf_id, floor_id)
        self.move_in_front_of_shelf(shelf_id)
        self.knowrob.finish_action()

        self.knowrob.start_looking_at_location(floor_id)
        if not self.knowrob.is_hanging_foor(floor_id):
            self.robosherlock.start_separator_detection(floor_id)
        else:
            self.robosherlock.start_mounting_bar_detection(floor_id)
        self.robosherlock.start_barcode_detection(shelf_id, floor_id)

        try:
            self.knowrob.start_move_to_shelf_frame_end()
            self.move_base.move_relative([-self.knowrob.get_floor_width(), 0, 0])
        except TimeoutError as e:
            self.move_base.STOP()


        barcodes = self.robosherlock.stop_barcode_detection()
        if not self.knowrob.is_hanging_foor(floor_id):
            separators = self.robosherlock.stop_separator_detection()
            self.knowrob.add_separators_and_barcodes(floor_id, separators, barcodes)
        else:
            mounting_bars = self.robosherlock.stop_separator_detection()
            self.knowrob.add_mounting_bars_and_barcodes(floor_id, mounting_bars, barcodes)
        self.knowrob.finish_action()
        self.knowrob.finish_action()
        self.knowrob.finish_action()

    def set_floor_scan_pose(self, shelf_id, floor_id):
        floor_position = self.knowrob.get_floor_position(floor_id)
        if self.knowrob.is_bottom_floor(floor_id):
            pose = FLOOR_SCAN_POSE_BOTTOM
        else:
            pose = FLOOR_SCAN_POSE_REST
        self.move_arm.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.move_arm.root),
                                                             Quaternion(*pose['rot'])))
        self.move_arm.set_translation_goal(
            PointStamped(Header(0, rospy.Time(), self.move_arm.root),
                         Point(pose['trans'][0],
                               pose['trans'][1] - floor_position.pose.position.y,
                               pose['trans'][2] + floor_position.pose.position.z)))
        self.move_arm.send_cartesian_goal()

    def move_in_front_of_shelf(self, shelf_id):
        return self.move_base.move_absolute_xyz(self.knowrob.get_perceived_frame_id(shelf_id),
                                                FLOOR_SCANNING_OFFSET['x'],
                                                FLOOR_SCANNING_OFFSET['y'],
                                                FLOOR_SCANNING_OFFSET['z'])

    def count_floor(self, shelf_id, floor_id):
        rospy.loginfo('counting objects on floor {}'.format(floor_id))
        self.knowrob.start_shelf_layer_counting()
        facings = self.knowrob.get_facings(floor_id)
        goal = deepcopy(COUNTING_OFFSET)
        goal.header.frame_id = self.knowrob.get_perceived_frame_id(floor_id)
        if self.knowrob.is_hanging_foor(floor_id):
            # TODO magic offset
            goal.pose.position.z = -goal.pose.position.z + 0.05
        goal = self.tf.transform_pose(self.move_arm.root, goal)
        goal.pose.position.x = COUNTING_OFFSET2
        self.move_arm.set_and_send_cartesian_goal(goal)
        if len(facings) == 0:
            self.move_base.move_relative([self.knowrob.get_floor_width(), 0, 0])
        else:
            frame_id = self.knowrob.get_perceived_frame_id(shelf_id)
            gripper_in_base = self.tf.lookup_transform(self.move_arm.root, self.move_arm.tip)
            for i, (facing_id, (facing_pose, product, width, left_sep)) in enumerate(
                    reversed(sorted(facings.items(), key=lambda (k, v): v[0].pose.position.x))):
                if i != 0:
                    self.knowrob.start_shelf_layer_counting()

                try:
                    self.move_base.move_absolute_xyz(frame_id,
                                                     gripper_in_base.pose.position.x + facing_pose.pose.position.x,
                                                     FLOOR_SCANNING_OFFSET['y'],
                                                     FLOOR_SCANNING_OFFSET['z'])
                except TimeoutError as e:
                    self.move_base.STOP()

                facing_type = 'hanging' if self.knowrob.is_hanging_foor(floor_id) else 'standing'
                count = self.robosherlock.count(product, width, left_sep, self.knowrob.get_perceived_frame_id(shelf_id), facing_type)
                for j in range(count):
                    self.knowrob.add_object(facing_id)
                rospy.loginfo('counted {} objects in facing {}'.format(count, facing_id))
                self.knowrob.finish_action()


    def STOP(self):
        self.move_base.STOP()
        self.move_arm.client.cancel_goal()
        self.move_arm.client.cancel_all_goals()


if __name__ == '__main__':
    rospy.init_node('brain')
    cram = CRAM()
    # cram.STOP()
    try:
        cmd = raw_input('start demo? [y]')
        if cmd == 'y':
            rospy.loginfo('starting REFILLS scenario 1 demo')
            cram.detect_baseboards()
            cram.start()
            rospy.spin()
            # cram.scan_shop()
            # rospy.loginfo('REFILLS scenario 1 demo completed')
    except Exception as e:
        traceback.print_exc()
    finally:
        rospy.loginfo('canceling all goals')
        cram.STOP()
        rospy.sleep(1)
