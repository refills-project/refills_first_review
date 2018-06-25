#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import QuaternionStamped, Quaternion, PointStamped, Point, PoseStamped, Pose
from sensor_msgs.msg import JointState
from refills_first_review.move_arm import GiskardWrapper
from refills_first_review.move_base import MoveBase

rospy.init_node('patrick_is_op')

arm = GiskardWrapper(enabled=True)
base = MoveBase(enabled=True)

joint_state = JointState()
joint_state.name = [
    'ur5_shoulder_pan_joint',
    'ur5_shoulder_lift_joint',
    'ur5_elbow_joint',
    'ur5_wrist_1_joint',
    'ur5_wrist_2_joint',
    'ur5_wrist_3_joint',
]
joint_state.position = [
    -1.81179792086,
    -1.79111653963,
    0.826759815216,
    -2.1456416289,
    -1.25228435198,
    1.56590378284,
]

arm.send_joint_goal(joint_state)

base_goal = PoseStamped()
base_goal.header.frame_id = 'map'
base_goal.pose.position = Point(0.983, -1.487, 0.000)
base_goal.pose.orientation = Quaternion(0.000, 0.000, 1.000, -0.012)

base.move_absolute(base_goal, retry=True)
