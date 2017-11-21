#!/usr/bin/env python
import rospy

from actionlib.simple_action_client import SimpleActionClient
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from giskard_msgs.msg._Controller import Controller
from giskard_msgs.msg._ControllerListAction import ControllerListAction
from giskard_msgs.msg._ControllerListGoal import ControllerListGoal
from sensor_msgs.msg._JointState import JointState


class Test(object):
    def __init__(self, action_server_name):
        # action server
        self.client = SimpleActionClient(action_server_name, ControllerListAction)
        self.client.wait_for_server()

    def send_cart_goal(self, goal_pose):
        goal = ControllerListGoal()
        goal.type = ControllerListGoal.STANDARD_CONTROLLER

        # translaiton
        controller = Controller()
        controller.type = Controller.TRANSLATION_3D
        controller.tip_link = 'gripper_tool_frame'
        controller.root_link = 'base_footprint'

        controller.goal_pose = goal_pose

        controller.p_gain = 3
        controller.enable_error_threshold = True
        controller.threshold_value = 0.05
        controller.weight = 1.0
        goal.controllers.append(controller)

        # rotation
        controller = Controller()
        controller.type = Controller.ROTATION_3D
        controller.tip_link = 'gripper_tool_frame'
        controller.root_link = 'base_footprint'

        controller.goal_pose = goal_pose

        controller.p_gain = 3
        controller.enable_error_threshold = True
        controller.threshold_value = 0.2
        controller.weight = 1.0
        goal.controllers.append(controller)

        self.client.send_goal(goal)
        result = self.client.wait_for_result(rospy.Duration(10))
        print('finished in 10s?: {}'.format(result))

    def send_joint_goal(self, joint_state):
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

if __name__ == '__main__':
    rospy.init_node('donbot_test_movements')

    test = Test(action_server_name='/move')

    joint_state = JointState()
    joint_state.name = ['ur5_shoulder_pan_joint',
                        'ur5_shoulder_lift_joint',
                        'ur5_elbow_joint',
                        'ur5_wrist_1_joint',
                        'ur5_wrist_2_joint',
                        'ur5_wrist_3_joint', ]
    joint_state.position = [0.0138173009806,
                            -0.948155685642,
                            1.64415424887,
                            0.833014565285,
                            -0.00345735390984,
                            -1.52754142152, ]
    test.send_joint_goal(joint_state)

    goal = PoseStamped()
    goal.header.frame_id = 'gripper_tool_frame'
    goal.pose.position.x = 0.1
    goal.pose.position.z = -0.2
    goal.pose.orientation.w = 1.0
    test.send_cart_goal(goal)

    goal = PoseStamped()
    goal.header.frame_id = 'gripper_tool_frame'
    goal.pose.position = Point(0,0,0)
    goal.pose.orientation = Quaternion(0.0 , 0.0, 0.70710678, 0.70710678)
    test.send_cart_goal(goal)

    goal = PoseStamped()
    goal.header.frame_id = 'gripper_tool_frame'
    goal.pose.position = Point(0,-0.1,0)
    goal.pose.orientation = Quaternion(-0.35355339, -0.35355339, -0.14644661,  0.85355339)
    test.send_cart_goal(goal)
