#!/usr/bin/env python

from __future__ import print_function
import rospy
import actionlib

from actionlib.simple_action_client import SimpleActionClient
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from giskard_msgs.msg._Controller import Controller
from giskard_msgs.msg._ControllerListAction import ControllerListAction
from giskard_msgs.msg._ControllerListGoal import ControllerListGoal
from move_base_msgs.msg._MoveBaseAction import MoveBaseAction
from move_base_msgs.msg._MoveBaseGoal import MoveBaseGoal
from sensor_msgs.msg._JointState import JointState
from std_msgs.msg._Header import Header
from tf.transformations import quaternion_from_euler
from numpy import pi


class MoveBase(object):
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.client = actionlib.SimpleActionClient('nav_pcontroller/move_base', MoveBaseAction)
        print('connecting ...')
        self.client.wait_for_server()
        print('connected')
        self.goal_pub = rospy.Publisher('move_base_goal', PoseStamped, queue_size=10)
        rospy.sleep(0.5)

    def __call__(self, target_pose):
        # self.client.cancel_all_goals()
        if self.enabled:
            self.goal_pub.publish(target_pose)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(30))
            result = self.client.get_result()
            rospy.loginfo('arrived at base goal {}'.format(result))
            return result

    def relative_pose(self, position, orientation):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'base_footprint'
        shelf.header = header
        shelf.pose.position = Point(*position)
        shelf.pose.orientation = Quaternion(*orientation)
        self(shelf)

    def goto_shelf1(self):
        # self.client.cancel_all_goals()
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'map'
        shelf.header = header
        shelf.pose.position = Point(0.135, 0.55, 0.0)
        shelf.pose.orientation = Quaternion(0.0, 0.0, 1.0, 0.0)
        self(shelf)

    def goto_shelf2(self):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'map'
        shelf.header = header
        shelf.pose.position = Point(1.085, 0.55, 0.0)
        shelf.pose.orientation = Quaternion(0.0, 0.0, 1.0, 0.0)
        self(shelf)

    def goto_shelf3(self):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'map'
        shelf.header = header
        shelf.pose.position = Point(2.085, 0.55, 0.000)
        shelf.pose.orientation = Quaternion(*quaternion_from_euler(0,0,pi*.98))
        self(shelf)

    def goto_shelf4(self):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'map'
        shelf.header = header
        shelf.pose.position = Point(2.95, 0.55, 0.000)
        shelf.pose.orientation = Quaternion(*quaternion_from_euler(0,0,pi*.99))
        self(shelf)


class MoveArm(object):
    def __init__(self, enabled):
        self.enabled = enabled
        self.client = SimpleActionClient('/qp_controller/command', ControllerListAction)
        rospy.loginfo('connecting to giskard')
        self.client.wait_for_server()
        rospy.loginfo('connected to giskard')
        self.tip = 'camera_link'
        self.root = 'base_footprint'
        self.joint_names = ['ur5_shoulder_pan_joint',
                            'ur5_shoulder_lift_joint',
                            'ur5_elbow_joint',
                            'ur5_wrist_1_joint',
                            'ur5_wrist_2_joint',
                            'ur5_wrist_3_joint', ]

    def send_cart_goal(self, goal_pose):
        if self.enabled:
            goal = ControllerListGoal()
            goal.type = ControllerListGoal.STANDARD_CONTROLLER

            # translation
            controller = Controller()
            controller.type = Controller.TRANSLATION_3D
            controller.tip_link = self.tip
            controller.root_link = self.root

            controller.goal_pose = goal_pose

            controller.p_gain = 3
            controller.enable_error_threshold = True
            controller.threshold_value = 0.05
            controller.weight = 1.0
            goal.controllers.append(controller)

            # rotation
            controller = Controller()
            controller.type = Controller.ROTATION_3D
            controller.tip_link = self.tip
            controller.root_link = self.root

            controller.goal_pose = goal_pose

            controller.p_gain = 3
            controller.enable_error_threshold = True
            controller.threshold_value = 0.2
            controller.weight = 1.0
            goal.controllers.append(controller)

            self.client.send_goal(goal)
            result = self.client.wait_for_result(rospy.Duration(10))
            print('finished in 10s?: {}'.format(result))

    def relative_goal(self, position, orientation):
        p = PoseStamped()
        p.header.frame_id = self.tip
        p.pose.position = Point(*position)
        p.pose.orientation = Quaternion(*orientation)
        self.send_cart_goal(p)

    def root_goal(self, position, orientation):
        p = PoseStamped()
        p.header.frame_id = 'base_footprint'
        p.pose.position = Point(*position)
        p.pose.orientation = Quaternion(*orientation)
        self.send_cart_goal(p)

    def send_joint_goal(self, joint_state):
        if self.enabled:
            goal = ControllerListGoal()
            goal.type = ControllerListGoal.STANDARD_CONTROLLER

            # translation
            controller = Controller()
            controller.type = Controller.JOINT
            controller.tip_link = 'gripper_tool_frame'
            controller.root_link = 'base_footprint'

            controller.goal_state = joint_state

            controller.p_gain = 3
            controller.enable_error_threshold = False
            controller.threshold_value = 0.05
            controller.weight = 1.0
            goal.controllers.append(controller)

            self.client.send_goal(goal)
            result = self.client.wait_for_result(rospy.Duration(10))
            print('finished in 10s?: {}'.format(result))

    def start_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [-0.807678524648,
                                -1.37526542345,
                                1.10520267487,
                                -2.51714307467,
                                -2.3022740523,
                                1.81282937527, ]
        self.send_joint_goal(joint_state)

    def start_pose_row4(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.53236753145,
            -1.5683611075,
            0.721428394318,
            -2.03346759478,
            -1.60843974749,
            1.58060956001,
        ]
        self.send_joint_goal(joint_state)

    def shelf1_row0_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.53295499483,
            -0.62881118456,
            2.41471195221,
            -3.84744292894,
            -1.60784131685,
            1.58060956001,
        ]
        self.send_joint_goal(joint_state)

    def shelf1_row1_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.53309852282,
            -0.638441387807,
            2.41393232346,
            -4.65580696264,
            -1.60776931444,
            1.58044183254,
        ]
        self.send_joint_goal(joint_state)

    def drive_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [-1.54014426867,
                                -2.51763660112,
                                1.38093948364,
                                -2.05715114275,
                                -1.57574254671,
                                1.5231782198, ]
        self.send_joint_goal(joint_state)


SHELF_LENGTH = 0.9


def scan_shelf1(move_arm, move_base):
    move_arm.drive_pose()
    move_base.goto_shelf1()

    # row4
    move_arm.start_pose_row4()
    move_base.relative_pose([-SHELF_LENGTH, 0, 0], [0, 0, 0, 1])

    # row 3
    move_arm.relative_goal([0., 0.2, 0], [0, 0, 0, 1])
    move_base.goto_shelf1()

    # row 2
    move_arm.relative_goal([0., 0.3, 0], [0, 0, 0, 1])
    move_base.relative_pose([-SHELF_LENGTH, 0, 0], [0, 0, 0, 1])

    # row 1
    move_arm.relative_goal([0., 0.35, 0], [0, 0, 0, 1])
    move_base.goto_shelf1()

    # row 0
    move_arm.shelf1_row0_pose()
    move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
    move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])


def scan_shelf2(move_arm, move_base):
    # row 0
    move_base.goto_shelf2()
    move_arm.shelf1_row0_pose()
    move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
    move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])
    move_base.relative_pose([0.0, 0.12, 0], [0, 0, 0, 1])

    # row 1
    move_arm.shelf1_row1_pose()
    move_arm.relative_goal([0., 0.15, 0], [0, 0, 0, 1])
    move_base.goto_shelf2()

    # row 2
    move_arm.relative_goal([0., -0.2, 0], [0, 0, 0, 1])
    move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

    # row 3
    move_arm.relative_goal([0., -0.5, 0], [0, 0, 0, 1])
    move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
    move_base.relative_pose([SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

    # row 4
    move_arm.relative_goal([0., -0.3, 0], [0, 0, 0, 1])
    move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])
    move_base.relative_pose([0.0, 0.12, 0], [0, 0, 0, 1])

def scan_shelf3(move_arm, move_base):
    pass
    # row 4
    move_base.goto_shelf3()
    move_arm.start_pose_row4()
    move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

    # row 3
    move_arm.relative_goal([0., 0.35, 0], [0, 0, 0, 1])
    move_base.goto_shelf3()

    # row 2
    move_arm.relative_goal([0., 0.3, 0], [0, 0, 0, 1])
    move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

    # row 1
    move_arm.relative_goal([0., 0.3, 0], [0, 0, 0, 1])
    move_base.goto_shelf3()

    # row 0
    move_arm.shelf1_row0_pose()
    move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
    move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])


def scan_shelf4(move_arm, move_base):
    to_close_to_wall_fix = 0.08
    short_shelf_length = SHELF_LENGTH
    # row 0
    move_base.goto_shelf4()
    move_arm.shelf1_row0_pose()
    move_arm.relative_goal([to_close_to_wall_fix, 0.0, 0], [0, 0, 0, 1])
    move_base.relative_pose([0.0, -0.1, 0], [0, 0, 0, 1])
    move_base.relative_pose([-short_shelf_length, 0.0, 0], [0, 0, 0, 1])

    # row 1
    move_base.relative_pose([0.0, 0.1, 0], [0, 0, 0, 1])
    move_arm.shelf1_row1_pose()
    move_arm.relative_goal([to_close_to_wall_fix, 0.1, 0], [0, 0, 0, 1])
    move_base.goto_shelf4()

    # row 2
    move_arm.relative_goal([0., -0.25, 0], [0, 0, 0, 1])
    move_base.relative_pose([-short_shelf_length, 0.0, 0], [0, 0, 0, 1])

    # row 3
    move_arm.relative_goal([0., -0.25, 0], [0, 0, 0, 1])
    move_base.goto_shelf4()

    # row 4
    move_arm.relative_goal([0., -0.25, 0], [0, 0, 0, 1])
    move_base.relative_pose([-short_shelf_length, 0.0, 0], [0, 0, 0, 1])

    # row 5
    move_arm.relative_goal([0., -0.2, 0], [0, 0, 0, 1])
    move_base.goto_shelf4()

    # back to default
    move_arm.drive_pose()


if __name__ == '__main__':
    rospy.init_node('brain')
    move_base = MoveBase(True)
    move_base.client.cancel_all_goals()
    move_arm = MoveArm(True)

    scan_shelf1(move_arm, move_base)
    scan_shelf2(move_arm, move_base)
    scan_shelf3(move_arm, move_base)
    scan_shelf4(move_arm, move_base)
