#!/usr/bin/env python

from __future__ import print_function, division

import threading
import traceback
from simplejson import OrderedDict
import numpy as np
from time import time

import rospy
from actionlib import SimpleActionServer
from geometry_msgs.msg import QuaternionStamped, Quaternion, PointStamped, Point, PoseStamped, Pose
from multiprocessing import TimeoutError

from gnomevfs._gnomevfs import CancelledError
from refills_msgs.msg._ScanningAction import ScanningAction
from refills_msgs.msg._ScanningGoal import ScanningGoal
from std_msgs.msg import Header

from refills_first_review.knowrob_wrapper import KnowRob
from refills_first_review.move_arm import GiskardWrapper
from refills_first_review.move_base import MoveBase
from refills_first_review.robosherlock_wrapper import RoboSherlock

# base
# shelf id
FLOOR_SCANNING_OFFSET = {'x': -0.18,
                         'y': -0.92,
                         'z': np.pi}

# shelf id
FLOOR_DETECTION_OFFSET = {'x': 0.5,
                          'y': -1.3,
                          'z': np.pi}
# arm
# trans in camera_link, rot in base_footprint
COUNTING_OFFSET = {'trans': [0.0, -0.1, -0.1],
                   'rot': [-0.000, 0.805, -0.593, -0.000]
                   # 'rot': [0, 0.7071, -0.7071, 0]
                   }

# in base_footprint
FLOOR_SCAN_POSE_BOTTOM = {'trans': [-.15, -.646, 0.177],
                          'rot': [0, 0.858, -0.514, 0]}
# in base_footprint
FLOOR_SCAN_POSE_REST = {'trans': [-.15, -.6, -0.0],
                        'rot': [-0.111, -0.697, 0.699, 0.111]}
SHELF_BASEBOARD = PoseStamped(Header(0, rospy.Time(), 'base_footprint'),
                              Pose(Point(-0.137, -0.68, 0.223),
                                   Quaternion(-0.000, 0.841, -0.541, 0.000)))


ACTION_NAME = 'scanning_action'

class CRAM(object):
    def __init__(self):
        # TODO use paramserver [low]
        # TODO live logging [high]
        # TODO SMS [high]
        self._as = SimpleActionServer(ACTION_NAME, ScanningAction, execute_cb=self.action_cb, auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        self.knowrob = KnowRob()
        self.robosherlock = RoboSherlock(self.knowrob)
        self.move_base = MoveBase(enabled=True)
        self.move_arm = GiskardWrapper(enabled=True)
        self.map_frame_id = rospy.get_param('~/map', 'map')

    def start(self):
        self._as.start()
        rospy.loginfo('waiting for scanning action goal')

    def preempt_cb(self):
        self.STOP()
        raise CancelledError('cancelled old goal')

    def action_cb(self, goal):
        if goal.type != ScanningGoal.COMPLETE_SCAN:
            rospy.logerr('only complete scans are supported')
            self._as.set_aborted('only complete scans are supported')
        try:
            self.move_arm.drive_pose()
            real_shelves = self.knowrob.get_shelves()
            real_shelf_keys = real_shelves.keys()
            for shelf_id in goal.loc_id:
                rospy.loginfo('scanning {}'.format(shelf_id))
                # TODO hack until knowrob is integrated into mock gui
                self.unofficial_shelf_id = int(shelf_id[-1])
                real_shelf_id = real_shelf_keys[self.unofficial_shelf_id]
                self.scan_shelf(real_shelf_id)
                # TODO feedback
            self._as.set_succeeded()
        except CancelledError as e:
            rospy.loginfo('preempted')

    def scan_shop(self):
        # TODO make sure that nothing is close [medium]
        self.move_arm.drive_pose()
        self.detect_baseboards()
        self.move_arm.drive_pose()
        # TODO we need a way to determine the order
        for unofficial_shelf_id, shelf_id in enumerate(self.knowrob.get_shelves()):
            self.unofficial_shelf_id = unofficial_shelf_id
            rospy.loginfo('scanning shelf system \'{}\''.format(shelf_id))
            t = time()
            self.scan_shelf(shelf_id)
            rospy.loginfo('scanned shelf system \'{}\' in {:.2f}s'.format(shelf_id, time() - t))

    def detect_baseboards(self):
        rospy.loginfo('shelf baseboard detection requires manuel mode')
        rospy.loginfo('move to free space plx')
        cmd = raw_input('done? [y]')
        if cmd == 'n':
            rospy.logwarn('skipping baseboard detection')
            self.robosherlock.baseboard_detection.detect_fake_shelves('0123')
            self.robosherlock.start_baseboard_detection()
        else:
            if cmd == 'y':
                rospy.loginfo('moving arm to baseboard scanning pose')
                self.move_arm.pre_baseboard_pose()
                self.move_arm.set_and_send_cartesian_goal(SHELF_BASEBOARD)
            else:
                raise UserWarning('you dumb... ABORT!!')
            rospy.loginfo('scan all shelf baseboard plx')

            self.robosherlock.start_baseboard_detection()

            cmd = raw_input('done? [y]')
            if cmd != 'y':
                raise UserWarning('you dumb... ABORT!!')

            # TODO check if shit is really save [medium]
            rospy.loginfo('MAKE SURE NOTHING IS CLOSE!!!!11elf')
            cmd = raw_input('rdy? [y]')
            if cmd != 'y' or self.move_base.is_stuff_close():
                raise UserWarning('you dumb... ABORT!!')

        shelves = self.robosherlock.stop_baseboard_detection()
        self.knowrob.add_shelves(shelves)

    def scan_shelf(self, shelf_id):
        self.detect_shelf_floors(shelf_id)
        for shelf_floor_id in self.knowrob.get_floor_ids(shelf_id):
            if not self.knowrob.is_floor_too_high(shelf_floor_id):
                self.scan_floor(shelf_id, shelf_floor_id)
                if not self.knowrob.is_hanging_foor(shelf_floor_id):
                    self.count_floor(shelf_id, shelf_floor_id)
        self.move_arm.drive_pose()

    def detect_shelf_floors(self, shelf_id):
        self.move_base.move_absolute_xyz(self.knowrob.get_object_frame_id(shelf_id),
                                         FLOOR_DETECTION_OFFSET['x'],
                                         FLOOR_DETECTION_OFFSET['y'],
                                         FLOOR_DETECTION_OFFSET['z'])
        self.robosherlock.start_floor_detection(shelf_id)
        self.move_arm.floor_detection_pose()
        if self.robosherlock.robosherlock:
            rospy.sleep(10)
        self.move_arm.floor_detection_pose2()
        if self.robosherlock.robosherlock:
            rospy.sleep(10)
        floor_heights = self.robosherlock.stop_floor_detection(shelf_id, self.unofficial_shelf_id)
        self.knowrob.add_shelf_floors(shelf_id, floor_heights)

    def scan_floor(self, shelf_id, floor_id):
        rospy.loginfo('scanning floor {}/{}'.format(shelf_id, floor_id))

        self.set_floor_scan_pose(shelf_id, floor_id)
        self.move_arm.send_cartesian_goal()
        self.move_in_front_of_shelf(shelf_id)

        if not self.knowrob.is_hanging_foor(floor_id):
            self.robosherlock.start_separator_detection(shelf_id, floor_id)
        self.robosherlock.start_barcode_detection(shelf_id, floor_id)

        try:
            self.move_base.move_relative([-self.knowrob.get_floor_width(), 0, 0])
        except TimeoutError as e:
            self.move_base.STOP()

        if not self.knowrob.is_hanging_foor(floor_id):
            separators = self.robosherlock.stop_separator_detection()
            self.knowrob.add_separators(shelf_id, floor_id, separators)
        barcodes = self.robosherlock.stop_barcode_detection()
        self.knowrob.add_barcodes(barcodes)

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

    def move_in_front_of_shelf(self, shelf_id):
        return self.move_base.move_absolute_xyz(self.knowrob.get_object_frame_id(shelf_id),
                                                FLOOR_SCANNING_OFFSET['x'],
                                                FLOOR_SCANNING_OFFSET['y'],
                                                FLOOR_SCANNING_OFFSET['z'])

    def count_floor(self, shelf_id, floor_id):
        rospy.loginfo('counting objects on floor {}/{}'.format(shelf_id, floor_id))

        facings = self.knowrob.get_facings(shelf_id, floor_id)
        self.move_arm.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.move_arm.root),
                                                             Quaternion(*COUNTING_OFFSET['rot'])))
        self.move_arm.set_translation_goal(PointStamped(Header(0, rospy.Time(), self.move_arm.tip),
                                                        Point(*COUNTING_OFFSET['trans'])))
        self.move_arm.send_cartesian_goal()
        if len(facings) == 0:
            self.move_base.move_relative([self.knowrob.get_floor_width(), 0, 0])
        else:
            frame_id = self.knowrob.get_object_frame_id(shelf_id)
            for i, facing_y in enumerate(reversed(sorted(facings))):
                self.move_base.move_absolute_xyz(frame_id,
                                                 FLOOR_SCANNING_OFFSET['x'] + facing_y,
                                                 FLOOR_SCANNING_OFFSET['y'],
                                                 FLOOR_SCANNING_OFFSET['z'])
                count = self.robosherlock.count()
                # TODO get name of object in facing [medium]
                rospy.loginfo('counted {} {} times'.format('muh', count))

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
