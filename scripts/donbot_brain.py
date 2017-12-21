#!/usr/bin/env python

from __future__ import print_function

from collections import OrderedDict
from multiprocessing import TimeoutError

import rospy
import actionlib

from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from giskard_msgs.msg._Controller import Controller
from giskard_msgs.msg._ControllerListAction import ControllerListAction
from giskard_msgs.msg._ControllerListGoal import ControllerListGoal
from move_base_msgs.msg._MoveBaseAction import MoveBaseAction
from move_base_msgs.msg._MoveBaseGoal import MoveBaseGoal
from refills_msgs.srv import CountObjects, CountObjectsRequest, CountObjectsResponse
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message, \
    convert_ros_message_to_dictionary
from sensor_msgs.msg._JointState import JointState
from std_msgs.msg._Header import Header
from tf.transformations import quaternion_from_euler
from numpy import pi

knowrob = OrderedDict([
    ('separators', [0.01, 0.161, .223, .285, .37, .512, .616, .758, .864, 0.98]),
    ('500183', [
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.01,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '500183',
         'obj_width': 140,
         'obj_height': 200,
         'obj_depth': 41},
    ]),
    ('046088', [
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.161,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '046088',
         'obj_width': 45,
         'obj_height': 150,
         'obj_depth': 40},
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.223,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '046088',
         'obj_width': 45,
         'obj_height': 150,
         'obj_depth': 40},
    ]),
    ('262289', [
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.285,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '262289',
         'obj_width': 45,
         'obj_height': 150,
         'obj_depth': 40},
    ]),
    ('010055', [
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.37,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '010055',
         'obj_width': 130,
         'obj_height': 210,
         'obj_depth': 45},
    ]),
    ('015652', [
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.512,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '015652',
         'obj_width': 75,
         'obj_height': 290,
         'obj_depth': 50},
    ]),
    ('516937', [
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.616,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '516937',
         'obj_width': 130,
         'obj_height': 200,
         'obj_depth': 100},
    ]),
    ('125481', [
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.758,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '125481',
         'obj_width': 90,
         'obj_height': 250,
         'obj_depth': 70},
        {'separator_location': {'header': {'stamp': {'secs': 0,
                                                     'nsecs': 0},
                                           'frame_id': '/map',
                                           'seq': 0},
                                'pose': {'position': {'x': -0.864,
                                                      'y': 0.515 - 0.1,
                                                      'z': 0.16 + 0.4},
                                         'orientation': {'y': 0.0,
                                                         'x': 0.0,
                                                         'z': 0.0,
                                                         'w': 1.0}}},
         'object_name': '125481',
         'obj_width': 90,
         'obj_height': 250,
         'obj_depth': 70},
    ]),
])


class MoveBase(object):
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.client = actionlib.SimpleActionClient('nav_pcontroller/move_base', MoveBaseAction)
        print('connecting ...')
        self.client.wait_for_server()
        print('connected')
        self.goal_pub = rospy.Publisher('move_base_goal', PoseStamped, queue_size=10)
        rospy.sleep(0.5)
        self.timeout = 30
        self.dist_to_shelfs = 1.4

    def __call__(self, target_pose):
        # self.client.cancel_all_goals()
        if self.enabled:
            self.goal_pub.publish(target_pose)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            self.client.send_goal(goal)
            wait_result = self.client.wait_for_result(rospy.Duration(self.timeout))
            result = self.client.get_result()
            state = self.client.get_state()
            if not wait_result or state != GoalStatus.SUCCEEDED:
                print('movement did not finish in time')
                raise TimeoutError()
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
        shelf.pose.position = Point(-0.0, self.dist_to_shelfs, 0.0)
        shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self(shelf)

    def goto_shelf2(self):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'map'
        shelf.header = header
        shelf.pose.position = Point(-1.0, self.dist_to_shelfs, 0.0)
        shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self(shelf)

    def goto_shelf3(self):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'map'
        shelf.header = header
        shelf.pose.position = Point(-2.0, self.dist_to_shelfs, 0.000)
        shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        # shelf.pose.orientation = Quaternion(*quaternion_from_euler(0,0,pi*.98))
        self(shelf)

    def goto_shelf4(self):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'map'
        shelf.header = header
        shelf.pose.position = Point(-2.9, self.dist_to_shelfs, 0.000)
        shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        # shelf.pose.orientation = Quaternion(*quaternion_from_euler(0,0,pi*.99))
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

    def counting_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [-1.60456067721,
                                -1.63770848909,
                                2.34750556946,
                                -3.24711829821,
                                -1.54815704027,
                                1.58525156975,]
        self.send_joint_goal(joint_state)


SHELF_LENGTH = 1.0

def scan_shelf(move_arm, move_base, shelf_position, shelf_heights):
    move_arm.drive_pose()
    move_base(shelf_position)
    move_arm.start_pose_row4()

    for shelf_height in shelf_heights:
        pass



class CRAM(object):
    def __init__(self):
        self.move_base = MoveBase(True)
        self.move_arm = MoveArm(True)
        self.counting_service = rospy.ServiceProxy('/count_objects_node/count', CountObjects)
        self.magic_offset = 0.05

    def scan_shelf1(self):
        self.move_arm.drive_pose()
        self.move_base.goto_shelf1()

        # row4
        self.move_arm.start_pose_row4()
        self.move_base.relative_pose([-SHELF_LENGTH, 0, 0], [0, 0, 0, 1])

        # row 3
        self.move_arm.relative_goal([0., 0.2, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf1()

        # row 2
        self.move_arm.relative_goal([0., 0.3, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-SHELF_LENGTH, 0, 0], [0, 0, 0, 1])

        # row 1
        self.move_arm.relative_goal([0., 0.35, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf1()

        # row 0
        self.move_arm.shelf1_row0_pose()
        self.move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

    def scan_shelf2(self):
        # row 0
        self.move_base.goto_shelf2()
        self.move_arm.shelf1_row0_pose()
        self.move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([0.0, 0.12, 0], [0, 0, 0, 1])

        # row 1
        self.move_arm.shelf1_row1_pose()
        self.move_arm.relative_goal([0., 0.15, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf2()

        # row 2
        self.move_arm.relative_goal([0., -0.2, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

        # row 3
        self.move_arm.relative_goal([0., -0.5, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

        # row 4
        self.move_arm.relative_goal([0., -0.3, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([0.0, 0.12, 0], [0, 0, 0, 1])

    def scan_shelf3(self):
        pass
        # row 4
        self.move_base.goto_shelf3()
        self.move_arm.start_pose_row4()
        self.move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

        # row 3
        self.move_arm.relative_goal([0., 0.35, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf3()

        # row 2
        self.move_arm.relative_goal([0., 0.3, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

        # row 1
        self.move_arm.relative_goal([0., 0.3, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf3()

        # row 0
        self.move_arm.shelf1_row0_pose()
        self.move_base.relative_pose([0.0, -0.12, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-SHELF_LENGTH, 0.0, 0], [0, 0, 0, 1])

    def scan_shelf4(self):
        to_close_to_wall_fix = 0.08
        short_shelf_length = SHELF_LENGTH - 0.14
        # row 0
        self.move_base.goto_shelf4()
        self.move_arm.shelf1_row0_pose()
        self.move_arm.relative_goal([to_close_to_wall_fix, 0.0, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([0.0, -0.1, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-short_shelf_length, 0.0, 0], [0, 0, 0, 1])

        # row 1
        self.move_base.relative_pose([0.0, 0.1, 0], [0, 0, 0, 1])
        self.move_arm.shelf1_row1_pose()
        self.move_arm.relative_goal([to_close_to_wall_fix, 0.1, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf4()

        # row 2
        self.move_arm.relative_goal([0., -0.25, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-short_shelf_length, 0.0, 0], [0, 0, 0, 1])

        # row 3
        self.move_arm.relative_goal([0., -0.25, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf4()

        # row 4
        self.move_arm.relative_goal([0., -0.25, 0], [0, 0, 0, 1])
        self.move_base.relative_pose([-short_shelf_length, 0.0, 0], [0, 0, 0, 1])

        # row 5
        self.move_arm.relative_goal([0., -0.2, 0], [0, 0, 0, 1])
        self.move_base.goto_shelf4()

        # back to default
        self.move_arm.drive_pose()

    def cancel_all_goals(self):
        self.move_base.client.cancel_goal()
        self.move_base.client.cancel_all_goals()
        self.move_arm.client.cancel_goal()
        self.move_arm.client.cancel_all_goals()

    def count(self, barcode, separator_id):
        global knowrob
        msg = CountObjectsRequest()
        msg.separator_location = convert_dictionary_to_ros_message('geometry_msgs/PoseStamped',
                                                                   knowrob[barcode][separator_id]['separator_location'])
        msg.separator_location.pose.position.y += self.magic_offset
        msg.separator_location.pose.position.z += self.magic_offset + 0.05
        msg.obj_depth = knowrob[barcode][0]['obj_depth']
        msg.obj_width = knowrob[barcode][0]['obj_width']
        msg.obj_height = knowrob[barcode][0]['obj_height']
        msg.object_name = knowrob[barcode][0]['object_name']

        next_item = knowrob['separators'].index(-msg.separator_location.pose.position.x) + 1
        next_x = - knowrob['separators'][next_item]
        counting_pose = PoseStamped()
        counting_pose.header.frame_id = 'map'
        counting_pose.pose.position = Point((msg.separator_location.pose.position.x + next_x) / 2 + 0.05,
                                            self.move_base.dist_to_shelfs,
                                            0.0)
        counting_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.move_base(counting_pose)
        print(msg)
        resp = self.counting_service.call(msg)
        rospy.loginfo('object with barcode "{}" counted {} times'.format(barcode, resp.object_count))
        return resp.object_count

    def counting(self):
        self.move_arm.drive_pose()
        self.move_base.goto_shelf1()
        self.move_arm.start_pose_row4()
        self.move_arm.counting_pose()

        total_objects = 0

        total_objects += self.count('500183', 0)
        total_objects += self.count('046088', 0)
        total_objects += self.count('046088', 1)
        total_objects += self.count('262289', 0)
        total_objects += self.count('010055', 0)
        total_objects += self.count('015652', 0)
        total_objects += self.count('516937', 0)
        total_objects += self.count('125481', 0)
        total_objects += self.count('125481', 1)
        print("total number of ojbects {}".format(total_objects))
        pass


if __name__ == '__main__':
    rospy.init_node('brain')
    cram = CRAM()
    cram.cancel_all_goals()
    try:
        cmd = raw_input('barcodes or counting? [b/c]')
        if cmd == 'b':
            print('starting barcode scanning')
            cram.scan_shelf1()
            cram.scan_shelf2()
            cram.scan_shelf3()
            cram.scan_shelf4()
        elif cmd == 'c':
            print('starting counting')
            cram.counting()
    except Exception as e:
        print('{}: {}'.format(e.__class__, e.message))
    finally:
        print('canceling all goals')
        cram.cancel_all_goals()
        rospy.sleep(1)
