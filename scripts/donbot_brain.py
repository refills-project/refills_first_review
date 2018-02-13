#!/usr/bin/env python

from __future__ import print_function, division

from simplejson import OrderedDict
import numpy as np
from time import time

import rospy
from geometry_msgs.msg import QuaternionStamped, Quaternion, PointStamped, Point, PoseStamped, Pose
from std_msgs.msg import Header
from tf.transformations import quaternion_about_axis

from refills_first_review.knowrob_wrapper import KnowRob
from refills_first_review.move_arm import GiskardWrapper
from refills_first_review.move_base import MoveBase
from refills_first_review.robosherlock_wrapper import RoboSherlock
from refills_first_review.separator_detection import SeparatorClustering

SHELF_LENGTH = 1.0


class CRAM(object):
    def __init__(self):
        self.knowrob = KnowRob()
        self.robosherlock = RoboSherlock()
        self.move_base = MoveBase(enabled=True)
        self.move_arm = GiskardWrapper(enabled=True)
        self.separator_detection = SeparatorClustering()
        self.magic_offset = 0.05

    def scan_shelf1(self):
        self.move_arm.drive_pose()
        self.move_base.goto_shelf1()

        # row4
        self.move_arm.start_pose_row4()
        self.move_base.move_relative([-SHELF_LENGTH, 0, 0], [0, 0, 0, 1])

        # row 3
        self.move_arm.relative_goal([0., 0.2, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf1()

        # row 2
        self.move_arm.relative_goal([0., 0.3, 0], [0, 0, 0, 1])
        self.move_base.move_relative([-SHELF_LENGTH, 0, 0], [0, 0, 0, 1])

        # row 1
        self.move_arm.relative_goal([0., 0.35, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf1()

        # row 0
        self.move_arm.shelf1_row0_pose()
        self.move_base.move_relative([0.0, -0.12, 0], [0, 0, 0, 1])
        self.move_base.move_relative([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

    def STOP(self):
        # TODO override old base goal
        self.move_base.client.cancel_goal()
        self.move_base.client.cancel_all_goals()
        self.move_arm.client.cancel_goal()
        self.move_arm.client.cancel_all_goals()

    def move_base_to_free_space(self):
        success = True
        return success

    def move_in_front_of_shelf(self, shelf_id):
        # TODO add some offset to shelf position
        x_offset = 0.6
        y_offset = 0.0
        rot_offset = -np.pi / 2

        base_goal = PoseStamped()
        base_goal.header.frame_id = self.knowrob.get_shelf_frame_id(shelf_id)
        base_goal.pose.position.x = x_offset
        base_goal.pose.position.y = y_offset
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(rot_offset, [0, 0, 1]))
        return self.move_base.move_absolute(base_goal)

    def go_into_floor_detection_pose(self, shelf_id):
        x_offset = 0.6
        y_offset = 0.3
        rot_offset = -np.pi / 2

        base_goal = PoseStamped()
        base_goal.header.frame_id = self.knowrob.get_shelf_frame_id(shelf_id)
        base_goal.pose.position.x = x_offset
        base_goal.pose.position.y = y_offset
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(rot_offset, [0, 0, 1]))
        success = self.move_base.move_absolute(base_goal)
        if not success:
            return False

        # TODO arm joint pose
        pass

    def detect_shelves(self):
        self.move_base_to_free_space()
        # TODO move arm into scanning pose
        # TODO drive around and call fussleisten detector
        # TODO put shelf position into knowrob
        # TODO return shelf positions
        return OrderedDict([('shelf1', PoseStamped(Header(0, rospy.Time(), 'map'),
                                                   Pose(Point(0, .515, 0),
                                                        Quaternion(0, 0, 0, 1)))),
                            ('shelf2', PoseStamped(Header(0, rospy.Time(), 'map'),
                                                   Pose(Point(-1, .515, 0),
                                                        Quaternion(0, 0, 0, 1)))),
                            ('shelf3', PoseStamped(Header(0, rospy.Time(), 'map'),
                                                   Pose(Point(-2, .515, 0),
                                                        Quaternion(0, 0, 0, 1)))),
                            ('shelf4', PoseStamped(Header(0, rospy.Time(), 'map'),
                                                   Pose(Point(-3, .515, 0),
                                                        Quaternion(0, 0, 0, 1))))])

    def detect_shelf_floors(self, shelf_id):
        self.go_into_floor_detection_pose(shelf_id)
        floor_heights = self.robosherlock.detect_floors(shelf_id)
        self.knowrob.add_shelf_floors(shelf_id, floor_heights)
        return floor_heights

    def scan_floor(self, shelf_id, floor_id):
        rospy.loginfo('scanning floor {}/{}'.format(shelf_id, floor_id))
        self.move_in_front_of_shelf(shelf_id)
        self.separator_detection.start_listening(shelf_id, floor_id)
        # TODO scan barcode
        self.set_floor_scan_pose(shelf_id, floor_id)
        self.move_arm.send_cartesian_goal()
        self.move_base.move_relative([-self.knowrob.get_floor_width(), 0, 0])
        # TODO do counting
        separators = self.separator_detection.stop_listening()
        self.knowrob.add_separators(separators)

    def count_floor(self, shelf_id, floor_id):
        rospy.loginfo('counting objects on floor {}/{}'.format(shelf_id, floor_id))
        x_offset = 0.0
        y_offset = 0.0
        z_offset = -0.1
        # TODO get facings
        facings = self.knowrob.get_facings(shelf_id, floor_id)
        # TODO loop over objects and count
        self.move_arm.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.move_arm.root),
                                                             Quaternion(0, 0.7071, -0.7071, 0)))
        self.move_arm.set_translation_goal(PointStamped(Header(0, rospy.Time(), self.move_arm.tip),
                                                        Point(x_offset,
                                                              y_offset,
                                                              z_offset)))
        self.move_arm.send_cartesian_goal()
        if len(facings) == 0:
            self.move_base.move_relative([self.knowrob.get_floor_width(), 0, 0])
        else:
            for i, facing_x in enumerate(reversed(sorted(facings))):
                target_base_pose = PoseStamped()
                target_base_pose.header.frame_id = self.knowrob.get_shelf_frame_id(shelf_id)
                target_base_pose.pose.position = Point(.6,facing_x, .0)
                target_base_pose.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi/2, [0, 0, 1]))
                self.move_base.move_absolute(target_base_pose)
                count = self.robosherlock.count()
                rospy.loginfo('counted {} {} times'.format('muh', count))

    def set_floor_scan_pose(self, shelf_id, floor_id):
        if floor_id == 0:
            x_offset = -0.1
            y_offset = -.3
            z_offset = 0.2
            self.move_arm.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.move_arm.root),
                                                                 Quaternion(0, 0.8398, -0.5428, 0)))
            self.move_arm.set_translation_goal(
                PointStamped(Header(0, rospy.Time(), self.move_arm.root),
                             Point(x_offset,
                                   y_offset,
                                   self.knowrob.get_floor_height(shelf_id, floor_id) + z_offset)))
        else:
            x_offset = -.1
            y_offset = -.3
            z_offset = 0.1
            self.move_arm.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.move_arm.root),
                                                                 Quaternion(0, 0.7071, -0.7071, 0)))
            self.move_arm.set_translation_goal(
                PointStamped(Header(0, rospy.Time(), self.move_arm.root),
                             Point(x_offset,
                                   y_offset,
                                   self.knowrob.get_floor_height(shelf_id, floor_id) + z_offset)))

    def scan_shelf(self, shelf_id):
        shelf_floor_heights = self.detect_shelf_floors(shelf_id)
        for shelf_floor_id, shelf_floor_height in enumerate(shelf_floor_heights):
            self.scan_floor(shelf_id, shelf_floor_id)
            self.count_floor(shelf_id, shelf_floor_id)
        success = True
        return success

    def scan_shop(self):
        self.move_base_to_free_space()
        self.move_arm.drive_pose()
        shelves = self.detect_shelves()
        self.move_base_to_free_space()
        self.move_arm.drive_pose()
        for shelf_id, shelf_position in shelves.items():
            rospy.loginfo('scanning shelf {}'.format(shelf_id))
            t = time()
            self.scan_shelf(shelf_id)
            rospy.loginfo('scanning shelf {} in {}s'.format(shelf_id, time()-t))


if __name__ == '__main__':
    rospy.init_node('brain')
    cram = CRAM()
    cram.STOP()
    # try:
    cmd = raw_input('start demo? [enter]')
    if cmd == '':
        rospy.loginfo('starting barcode scanning')
        cram.scan_shop()
        rospy.loginfo('scanning completed')
    # except Exception as e:
    #     rospy.loginfo('{}: {}'.format(e.__class__, e))
    # finally:
    #     rospy.loginfo('canceling all goals')
    #     cram.STOP()
    #     rospy.sleep(1)
