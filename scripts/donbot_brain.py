#!/usr/bin/env python

from __future__ import print_function, division

import traceback
import numpy as np
from pymongo import MongoClient
from time import time
import datetime
import os
from subprocess import call

import PyKDL
import rospy
from rospkg import RosPack
from actionlib import SimpleActionServer
from copy import deepcopy
from geometry_msgs.msg import QuaternionStamped, Quaternion, PointStamped, Point, PoseStamped, Pose
from multiprocessing import TimeoutError

from refills_msgs.msg import ScanningFeedback
from refills_msgs.msg._ScanningAction import ScanningAction
from refills_msgs.msg._ScanningGoal import ScanningGoal
# from saving_images.srv import saveImage, saveImageRequest

from std_msgs.msg import Header
from tf.transformations import quaternion_conjugate, quaternion_about_axis

from refills_first_review.knowrob_wrapper import KnowRob
from refills_first_review.move_arm import GiskardWrapper
from refills_first_review.move_base import MoveBase
from refills_first_review.robosherlock_wrapper import RoboSherlock

# base
# shelf id
from refills_first_review.tfwrapper import lookup_transform, transform_pose, lookup_transform2
from refills_first_review.utils import kdl_to_pose

dist_to_shelf = 0.25

# in shelf_id
FLOOR_SCANNING_OFFSET = {'x': -0.34,
                         'y': -(1.085 + dist_to_shelf),
                         'z': np.pi}

# shelf floor detection params
# shelf id
# FLOOR_DETECTION_OFFSET = {'x': 0.4,
#                           'y': -(1.28),
#                           'z': np.pi}

# FLOOR_DETECTION_END_POSE = PoseStamped(Header(0, rospy.Time(), 'camera_link'),
#                                        Pose(Point(0,0.8,0),
#                                             Quaternion(0,0,0,1)))

# arm
# trans in camera_link, rot in base_footprint
# COUNTING_OFFSET = {'trans': [0.0, -0.15, -0.1],
#                    'rot': [-0.000, 0.805, -0.593, -0.000]
#                    }
# in floor_id
COUNTING_OFFSET = PoseStamped(Header(0, rospy.Time(), ''),
                              Pose(Point(0.097, -(0.322 + dist_to_shelf), 0.24),
                                   Quaternion(-0.771, -0.000, -0.000, 0.637)))
COUNTING_OFFSET2 = -0.54

# in base_footprint
FLOOR_SCAN_POSE_BOTTOM = {'trans': [-.512, -1.022, 0.05],
                          'rot': [0.007, 0.869, -0.495, -0.002]}
# in base_footprint
# FLOOR_SCAN_POSE_REST = {'trans': [-.15, -.645, -0.0],
#                         'rot': [-0.111, -0.697, 0.699, 0.111]}
FLOOR_SCAN_POSE_REST = {'trans': [-.512, -.96, -0.182],
                        'rot': [-0.007, -0.684, 0.729, 0.000]}
SHELF_BASEBOARD = PoseStamped(Header(0, rospy.Time(), 'base_link'),
                              Pose(Point(-0.497, -0.970, 0.063),
                                   Quaternion(-0.000, 0.841, -0.541, 0.000)))

ACTION_NAME = 'scanning_action'


# rs_camera_color_optical_frame
class CRAM(object):
    def __init__(self):
        # TODO use paramserver [low]
        self.mongo_client = MongoClient('localhost')
        self._as = SimpleActionServer(ACTION_NAME, ScanningAction, execute_cb=self.action_cb, auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        try:
            from refills_counting_image_saver.srv import saveImage, saveImageRequest
            self.save_image_srv = rospy.ServiceProxy('/saving_image', saveImage)
        except:
            pass
        self.knowrob = KnowRob()
        self.robosherlock = RoboSherlock(self.knowrob)
        self.move_base = MoveBase(enabled=True, knowrob=self.knowrob)
        self.move_arm = GiskardWrapper(enabled=True, knowrob=self.knowrob)
        self.map_frame_id = rospy.get_param('~/map', 'map')

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
            try:
                self.mongo_wipe()
            except OSError as exc:  # Python >2.5
                rospy.logwarn('failed to wipe mongo, IO error')

            self.move_arm.drive_pose()
            # TODO scan shelf system
            # self.knowrob.start_shelf_system_mapping(self.sys)
            for i, shelf_id in enumerate(goal.loc_id):
                rospy.loginfo('scanning {}'.format(shelf_id))
                self.scan_shelf(shelf_id)
                # TODO feedback
            self.knowrob.finish_action()

            try:
                rospy.loginfo('DONE')
                rospy.loginfo('exporting logs')
                data_path = '{}/data/'.format(RosPack().get_path('refills_first_review'))
                episode_name = str(datetime.date.today()) + '_' + str(time())
                episode_dir = data_path + episode_name
                os.makedirs(episode_dir)
                self.knowrob.save_beliefstate(episode_dir + '/beliefstate.owl')
                self.knowrob.save_action_graph(episode_dir + '/actions.owl')
                rospy.loginfo('logs exported')
                self.mongo_save(episode_dir)
            except OSError as exc:  # Python >2.5
                rospy.logwarn('failed to export logs, IO error')
            # self.knowrob.save_beliefstate()
            # self.knowrob.save_action_graph()

            self._as.set_succeeded()
        except Exception as e:
            traceback.print_exc()
            rospy.loginfo('preempted')
        rospy.loginfo('waiting for scanning action goal')

    def mongo_save(self, out_dir):
        os.system('mongodump --db REFILLS_0 --collection tf --out {}'.format(out_dir))
        # call(['mongodump',
        #       '--db', 'REFILLS_0',
        #       '--collection', 'tf',
        #       '--out', out_dir])

    def mongo_wipe(self):
        self.mongo_client.drop_database('REFILLS_0')

    def detect_baseboards(self):
        rospy.loginfo('shelf baseboard detection requires manuel mode')
        rospy.loginfo('move to free space plx, THE ARM WILL MOVE!!!')
        cmd = raw_input('done? [y/n]')
        while True:
            input = raw_input('add new shelf system? [y/n]')
            if input != 'y' and input != '1337':
                break
            shelf_system_id = self.knowrob.add_shelf_system()
            rospy.loginfo('added shelf system {}'.format(shelf_system_id))
            rospy.loginfo('moving arm to baseboard scanning pose')
            if input != '1337':
                self.goto_baseboard_detection_pose()
                rospy.loginfo('scan QR codes plx')
            self.robosherlock.start_baseboard_detection()
            if input == '1337':
                cmd = input
            else:
                cmd = 'n'
            while cmd != 'y' and cmd != '1337':
                cmd = raw_input('finished scanning shelf system? [y/n]')
            if cmd == '1337':
                rospy.logwarn('skipping baseboard detection')
                self.robosherlock.baseboard_detection.detect_fake_shelves('0123')

            shelves = self.robosherlock.stop_baseboard_detection()
            self.knowrob.add_shelves(shelf_system_id, shelves)
        rospy.loginfo('MAKE SURE NOTHING IS CLOSE!!!! THE ARM WILL MOVE')
        cmd = raw_input('rdy? [y/n]')
        if cmd == 'y':
            self.move_arm.drive_pose()
        else:
            raise UserWarning('learn to press \'y\' n00b')

    def goto_baseboard_detection_pose(self):
        self.move_arm.pre_baseboard_pose()
        goal = lookup_transform('base_footprint', self.move_arm.tip)
        goal.pose.position.z = 0.22
        self.move_arm.set_and_send_cartesian_goal(goal)

    def scan_shelf(self, shelf_id):
        feedback = ScanningFeedback()
        feedback.current_loc_id = shelf_id
        feedback.progress = 0
        self._as.publish_feedback(feedback)

        self.knowrob.start_shelf_frame_mapping(shelf_id)

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

    def detect_shelf_floors(self, shelf_system_id):
        self.knowrob.start_finding_shelf_layer()
        self.robosherlock.start_floor_detection(shelf_system_id)
        self.move_arm.floor_detection_pose()

        self.knowrob.start_move_to_shelf_frame(shelf_system_id)

        width = self.knowrob.get_shelf_system_width(shelf_system_id)
        self.move_to_shelf(shelf_system_id, width/2)

        self.knowrob.finish_action()

        self.knowrob.start_looking_at_location(shelf_system_id)
        if self.robosherlock.robosherlock:
            rospy.sleep(0)
        self.knowrob.finish_action()
        next_goal = PoseStamped()
        next_goal.header.frame_id = 'camera_link'
        next_goal.pose.orientation.w = 1
        next_goal.pose.position.y = 0.2
        for i in range(5):
            self.knowrob.start_looking_at_location(shelf_system_id)
            self.move_arm.set_and_send_cartesian_goal(next_goal)
            if self.robosherlock.robosherlock:
                rospy.sleep(0)
            self.knowrob.finish_action()

        self.move_to_shelf(shelf_system_id, width / 2, -0.6)
        rospy.sleep(4)

        floor_heights = self.robosherlock.stop_floor_detection(shelf_system_id)
        self.knowrob.add_shelf_layers(shelf_system_id, floor_heights)
        floor_ids = self.knowrob.get_floor_ids(shelf_system_id)
        self.knowrob.start_shelf_layer_perception(floor_ids)
        self.knowrob.finish_action()
        self.knowrob.finish_action()
        return floor_ids

    def get_floor_detection_base_pose_camera(self, shelf_system_id):
        width = self.knowrob.get_shelf_system_width(shelf_system_id)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.knowrob.get_perceived_frame_id(shelf_system_id)
        goal_pose.pose.position.x = width / 2
        goal_pose.pose.position.y = -0.5
        goal_pose.pose.orientation = Quaternion(*quaternion_about_axis(0, [0, 0, 1]))
        return goal_pose

    def shelf_layer_mapping(self, shelf_id, floor_id):
        self.knowrob.start_shelf_layer_mapping(floor_id)
        self.scan_floor(shelf_id, floor_id)
        # if not self.knowrob.is_hanging_foor(shelf_floor_id):
        self.count_floor(shelf_id, floor_id)
        self.knowrob.finish_action()

    def scan_floor(self, shelf_system_id, floor_id):
        rospy.loginfo('scanning floor {}'.format(floor_id))
        self.knowrob.start_finding_shelf_layer_parts()

        self.knowrob.start_move_to_shelf_layer()

        self.set_floor_scan_pose(shelf_system_id, floor_id)
        if self.knowrob.is_bottom_floor(floor_id):
            self.move_to_shelf_left(shelf_system_id, -0.58)
        else:
            self.move_to_shelf_left(shelf_system_id)
        self.knowrob.finish_action()

        self.knowrob.start_looking_at_location(floor_id)
        if not self.knowrob.is_hanging_foor(floor_id):
            self.robosherlock.start_separator_detection(floor_id)
        else:
            self.robosherlock.start_mounting_bar_detection(floor_id)
        self.robosherlock.start_barcode_detection(shelf_system_id, floor_id)

        try:
            self.knowrob.start_move_to_shelf_frame_end()
            if self.knowrob.is_bottom_floor(floor_id):
                self.move_to_shelf_right(shelf_system_id, -0.58)
            else:
                self.move_to_shelf_right(shelf_system_id)
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

    def set_floor_scan_pose(self, shelf_system_id, shelf_layer_id):
        orientation_goal = QuaternionStamped()
        orientation_goal.header.frame_id = self.move_arm.tip
        if self.knowrob.is_bottom_floor(shelf_layer_id):
            orientation_goal = self.get_tilt_orientation_goal(0.0)
        else:
            orientation_goal = self.get_tilt_orientation_goal(0.0)

        t_base_footprint___camera = lookup_transform('base_footprint', self.move_arm.tip)
        t_base_footprint___layer = lookup_transform('base_footprint',
                                                    self.knowrob.get_perceived_frame_id(shelf_layer_id))
        t_base_footprint___camera.pose.position.z = t_base_footprint___layer.pose.position.z
        trans_goal = PointStamped()
        trans_goal.header.frame_id = 'base_footprint'
        trans_goal.point = t_base_footprint___camera.pose.position

        if self.knowrob.is_bottom_floor(shelf_layer_id):
            trans_goal.point.z += 0.1
        else:
            trans_goal.point.z += 0.05

        self.move_arm.set_orientation_goal(orientation_goal)
        self.move_arm.set_translation_goal(trans_goal)
        self.move_arm.send_cartesian_goal()

    def move_to_shelf_left(self, shelf_system_id, y=-0.5):
        self.move_to_shelf(shelf_system_id, 0.1, y)

    def move_to_shelf_right(self, shelf_system_id, y=-0.5):
        width = self.knowrob.get_shelf_system_width(shelf_system_id)
        self.move_to_shelf(shelf_system_id, width - 0.18, y)

    def move_to_shelf(self, shelf_system_id, x, y=-0.5):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.knowrob.get_perceived_frame_id(shelf_system_id)
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = Quaternion(*quaternion_about_axis(0, [0, 0, 1]))
        self.move_base.move_absolute_link(goal_pose)

    def count_floor(self, shelf_system_id, shelf_layer_id):
        # TODO do absolute base movements

        rospy.loginfo('counting objects on floor {}'.format(shelf_layer_id))
        self.knowrob.start_shelf_layer_counting()
        facings = self.knowrob.get_facings(shelf_layer_id)
        # goal = deepcopy(COUNTING_OFFSET)
        # goal.header.frame_id = self.knowrob.get_perceived_frame_id(floor_id)
        # if self.knowrob.is_hanging_foor(floor_id):
        #     TODO magic offset
        # goal.pose.position.z = -goal.pose.position.z + 0.25
        # goal = transform_pose(self.move_arm.root, goal)
        # goal.pose.position.x = COUNTING_OFFSET2
        # self.move_arm.set_and_send_cartesian_goal(goal)
        if len(facings) == 0:
            # self.move_base.move_relative([self.knowrob.get_floor_width(), 0, 0])
            self.move_to_shelf_left(shelf_system_id)
        else:
            # frame_id = self.knowrob.get_perceived_frame_id(shelf_system_id)
            # gripper_in_base = lookup_transform('base_footprint', self.move_arm.tip)
            for i, (facing_id, (facing_pose, product, width, left_sep)) in enumerate(
                    reversed(sorted(facings.items(), key=lambda (k, v): v[0].pose.position.x))):
                if i != 0:
                    self.knowrob.start_shelf_layer_counting()

                try:
                    # self.move_base.move_absolute_xyz(frame_id,
                    #                                  gripper_in_base.pose.position.x + facing_pose.pose.position.x,
                    #                                  FLOOR_SCANNING_OFFSET['y'],
                    #                                  FLOOR_SCANNING_OFFSET['z'], retry=False)
                    self.goto_counting_position(shelf_system_id, shelf_layer_id, facing_id)
                except TimeoutError as e:
                    traceback.print_exc()
                    self.move_base.STOP()

                # facing_type = 'hanging' if self.knowrob.is_hanging_foor(shelf_layer_id) else 'standing'
                rospy.sleep(1)
                # count = self.robosherlock.count(product, width, left_sep, self.knowrob.get_perceived_frame_id(shelf_id),
                #                                 facing_type)
                count = self.robosherlock.count(facing_id, self.knowrob.get_perceived_frame_id(shelf_system_id))
                try:
                    self.save_image_srv.call(saveImageRequest(NameForPicture=int(product.split('AN')[1])))
                    rospy.loginfo('image saved')
                except:
                    pass
                for j in range(count):
                    self.knowrob.add_object(facing_id)
                rospy.loginfo('counted {} objects in facing {}'.format(count, facing_id))
                self.knowrob.finish_action()

    def goto_counting_position(self, shelf_system_id, shelf_layer_id, facing_id):
        p_base_footprint___layer = lookup_transform('base_footprint',
                                                    self.knowrob.get_perceived_frame_id(shelf_layer_id))
        self.move_arm.set_orientation_goal(self.get_tilt_orientation_goal(-0.35))
        self.move_arm.set_translation_goal(self.get_height_goal(p_base_footprint___layer.pose.position.z + 0.25))
        self.move_arm.send_cartesian_goal()

        p_layer___facing = lookup_transform(self.knowrob.get_perceived_frame_id(shelf_system_id),
                                            self.knowrob.get_object_frame_id(facing_id))
        self.move_to_shelf(shelf_system_id, p_layer___facing.pose.position.x)

    def get_tilt_orientation_goal(self, tilt):
        orientation_goal = QuaternionStamped()
        orientation_goal.header.frame_id = 'base_footprint'
        t_base_footprint___camera = PyKDL.Frame(PyKDL.Rotation(1, 0, 0,
                                                               0, 0, 1,
                                                               0, -1, 0))
        t_camera___camera_goal = PyKDL.Frame(PyKDL.Rotation.RotX(tilt))
        t_base_footprint___camera_goal = t_base_footprint___camera * t_camera___camera_goal
        orientation_goal.quaternion = kdl_to_pose(t_base_footprint___camera_goal).orientation
        return orientation_goal

    def get_height_goal(self, height):
        t_base_footprint___camera = lookup_transform('base_footprint', self.move_arm.tip)
        trans_goal = PointStamped()
        trans_goal.header.frame_id = 'base_footprint'
        trans_goal.point = t_base_footprint___camera.pose.position
        trans_goal.point.z = height
        return trans_goal

    def STOP(self):
        self.move_base.STOP()
        # self.move_arm.client.cancel_goal()
        # self.move_arm.client.cancel_all_goals()


if __name__ == '__main__':
    rospy.init_node('brain')
    cram = CRAM()
    # cram.STOP()
    try:
        cmd = raw_input('start demo? [y/n]')
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
