#!/usr/bin/env python

from __future__ import print_function, division

from simplejson import OrderedDict
import numpy as np
from time import time

import rospy
from geometry_msgs.msg import QuaternionStamped, Quaternion, PointStamped, Point, PoseStamped, Pose
from std_msgs.msg import Header
from tf.transformations import quaternion_about_axis

from refills_first_review.baseboard_detection import BaseboardDetector
from refills_first_review.knowrob_wrapper import KnowRob
from refills_first_review.move_arm import GiskardWrapper
from refills_first_review.move_base import MoveBase
from refills_first_review.robosherlock_wrapper import RoboSherlock
from refills_first_review.separator_detection import SeparatorClustering

# base
FLOOR_SCANNING_OFFSET = {'x': 0.95,
                         'y': -0.15,
                         'z': -np.pi / 2}
FLOOR_DETECTION_OFFSET = {'x': 1.3,
                          'y': 0.5,
                          'z': -np.pi / 2}
# arm
COUNTING_OFFSET = {'trans': [0.0, -0.1, -0.1],
                   'rot': [0, 0.7071, -0.7071, 0]}
FLOOR_SCAN_POSE_BOTTOM = {'trans': [-.15, -.646, 0.327],
                          'rot': [0, 0.858, -0.514, 0]}
FLOOR_SCAN_POSE_REST = {'trans': [-.15, -.7, 0.0],
                        'rot': [0, 0.7071, -0.7071, 0]}
SHELF_FUSSLEISTE = PoseStamped(Header(0, rospy.Time(), 'base_footprint'),
                               Pose(Point(-0.137, -0.68, 0.223),
                                    Quaternion(-0.000, 0.841, -0.541, 0.000)))


class CRAM(object):
    def __init__(self):
        self.knowrob = KnowRob()
        self.robosherlock = RoboSherlock()
        self.move_base = MoveBase(enabled=True)
        self.move_arm = GiskardWrapper(enabled=True)
        self.separator_detection = SeparatorClustering()
        self.fussleisten_detection = BaseboardDetector()
        self.map_frame_id = 'map'

    def STOP(self):
        self.move_base.STOP()
        self.move_arm.client.cancel_goal()
        self.move_arm.client.cancel_all_goals()

    def move_in_front_of_shelf(self, shelf_id):
        return self.move_base.move_absolute_xyz(self.knowrob.get_shelf_frame_id(shelf_id),
                                                FLOOR_SCANNING_OFFSET['x'],
                                                FLOOR_SCANNING_OFFSET['y'],
                                                FLOOR_SCANNING_OFFSET['z'])

    def go_into_floor_detection_pose(self, shelf_id):
        success = self.move_base.move_absolute_xyz(self.knowrob.get_shelf_frame_id(shelf_id),
                                                   FLOOR_DETECTION_OFFSET['x'],
                                                   FLOOR_DETECTION_OFFSET['y'],
                                                   FLOOR_DETECTION_OFFSET['z'])
        if not success:
            return False
        success = self.move_arm.floor_detection_pose()
        return success

    def detect_shelves(self):
        rospy.loginfo('searching for shelves')
        rospy.loginfo('switching to manuel mode')
        rospy.loginfo('move to free space plx')
        cmd = raw_input('done? [y]')
        if cmd == 'y':
            rospy.loginfo('moving arm to fussleisten scanning pose')
            # self.move_arm.floor_detection_pose()
            self.move_arm.pre_floor0_pose()
            self.move_arm.set_and_send_cartesian_goal(SHELF_FUSSLEISTE)
        else:
            raise UserWarning('you dumb')
        rospy.loginfo('scan all shelf fussleisten plx')

        self.fussleisten_detection.start_listening()

        cmd = raw_input('done? [y]')
        if cmd != 'y':
            raise UserWarning('you dumb')

        # TODO end shelf detector
        result = self.fussleisten_detection.stop_listening()
        # TODO put shelf position into knowrob
        return result

    def detect_shelf_floors(self, shelf_id):
        self.go_into_floor_detection_pose(shelf_id)
        floor_heights = self.robosherlock.detect_floors(shelf_id)
        self.knowrob.add_shelf_floors(shelf_id, floor_heights)
        return len(floor_heights)

    def scan_floor(self, shelf_id, floor_id):
        rospy.loginfo('scanning floor {}/{}'.format(shelf_id, floor_id))
        self.set_floor_scan_pose(shelf_id, floor_id)
        self.move_arm.send_cartesian_goal()
        self.move_in_front_of_shelf(shelf_id)
        self.separator_detection.start_listening(shelf_id, floor_id)
        # TODO scan barcode
        self.move_base.move_relative([-self.knowrob.get_floor_width(), 0, 0])
        separators = self.separator_detection.stop_listening()
        self.knowrob.add_separators(separators)
        return True

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
            for i, facing_x in enumerate(reversed(sorted(facings))):
                self.move_base.move_absolute_xyz(self.knowrob.get_shelf_frame_id(shelf_id),
                                                 FLOOR_SCANNING_OFFSET['x'],
                                                 FLOOR_SCANNING_OFFSET['y'] + facing_x,
                                                 FLOOR_SCANNING_OFFSET['z'])
                count = self.robosherlock.count()
                rospy.loginfo('counted {} {} times'.format('muh', count))
        return True

    def set_floor_scan_pose(self, shelf_id, floor_id):
        if floor_id == 0:
            self.move_arm.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.move_arm.root),
                                                                 Quaternion(*FLOOR_SCAN_POSE_BOTTOM['rot'])))
            self.move_arm.set_translation_goal(
                PointStamped(Header(0, rospy.Time(), self.move_arm.root),
                             Point(FLOOR_SCAN_POSE_BOTTOM['trans'][0],
                                   FLOOR_SCAN_POSE_BOTTOM['trans'][1],
                                   FLOOR_SCAN_POSE_BOTTOM['trans'][2])))
        else:

            self.move_arm.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.move_arm.root),
                                                                 Quaternion(*FLOOR_SCAN_POSE_REST['rot'])))
            self.move_arm.set_translation_goal(
                PointStamped(Header(0, rospy.Time(), self.move_arm.root),
                             Point(FLOOR_SCAN_POSE_REST['trans'][0],
                                   FLOOR_SCAN_POSE_REST['trans'][1],
                                   FLOOR_SCAN_POSE_REST['trans'][2] + self.knowrob.get_floor_height(shelf_id, floor_id))))

    def scan_shelf(self, shelf_id):
        number_of_floors = self.detect_shelf_floors(shelf_id)
        for shelf_floor_id in range(number_of_floors):
            if not self.knowrob.is_floor_too_high(shelf_id, shelf_floor_id):
                if not self.scan_floor(shelf_id, shelf_floor_id):
                    break
                if not self.count_floor(shelf_id, shelf_floor_id):
                    break
        else:  # if not break
            self.move_arm.drive_pose()
            return True
        return False

    def scan_shop(self):
        self.move_arm.drive_pose()
        shelves = self.detect_shelves()
        self.move_arm.drive_pose()
        for shelf_id, shelf_position in shelves.items():
            rospy.loginfo('scanning shelf \'{}\''.format(shelf_id))
            t = time()
            self.scan_shelf(shelf_id)
            rospy.loginfo('scanned shelf \'{}\' in {}s'.format(shelf_id, time() - t))


if __name__ == '__main__':
    rospy.init_node('brain')
    cram = CRAM()
    # cram.STOP()
    try:
        cmd = raw_input('start demo? [enter]')
        if cmd == '':
            rospy.loginfo('starting REFILLS scenario 1 demo')
            cram.scan_shop()
            rospy.loginfo('REFILLS scenario 1 demo completed')
    except Exception as e:
        #TODO does not work, fix it!
        rospy.loginfo('{}: {}'.format(e.__class__, e))
    finally:
        rospy.loginfo('canceling all goals')
        cram.STOP()
        rospy.sleep(1)
