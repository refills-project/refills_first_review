import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Quaternion, Point, PoseStamped, QuaternionStamped, PointStamped
from giskard_msgs.msg import Controller, ControllerListGoal, ControllerListAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class GiskardWrapper(object):
    def __init__(self, giskard_action_name='/qp_controller/command', enabled=True):
        self.move_time_limit = 20
        self.enabled = enabled
        self.client = SimpleActionClient(giskard_action_name, ControllerListAction)
        rospy.loginfo('connecting to {}'.format(giskard_action_name))
        self.client.wait_for_server()
        rospy.loginfo('connected to {}'.format(giskard_action_name))
        self.tip = 'camera_link'
        self.root = 'base_footprint'

        self.muh = rospy.Publisher('giskard_goal_pose', PoseStamped, queue_size=10)

        # TODO get this from param server of topic
        self.joint_names = ['ur5_shoulder_pan_joint',
                            'ur5_shoulder_lift_joint',
                            'ur5_elbow_joint',
                            'ur5_wrist_1_joint',
                            'ur5_wrist_2_joint',
                            'ur5_wrist_3_joint', ]

        current_pose = PoseStamped()
        current_pose.header.frame_id = self.tip
        current_pose.pose.orientation.w = 1
        self.set_translation_goal(current_pose)
        self.set_orientation_goal(current_pose)

    def set_orientation_goal_to_current(self):
        self.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), self.tip),
                                                    Quaternion(0, 0, 0, 1)))

    def set_translation_goal_to_current(self):
        self.set_translation_goal(PointStamped(Header(0, rospy.Time(), self.tip),
                                               Point(0, 0, 0)))

    def set_translation_goal(self, translation, weight=1.0):
        goal_pose = PoseStamped()
        if isinstance(translation, PointStamped):
            goal_pose.header = translation.header
            goal_pose.pose.position = translation.point
        else:
            goal_pose = translation
        self.translation_goal = self.make_controller(Controller.TRANSLATION_3D, goal_pose, weight)

    def set_orientation_goal(self, orientation, weight=1.0):
        goal_pose = PoseStamped()
        if isinstance(orientation, QuaternionStamped):
            goal_pose.header = orientation.header
            goal_pose.pose.orientation = orientation.quaternion
        else:
            goal_pose = orientation
        self.orientation_goal = self.make_controller(Controller.ROTATION_3D, goal_pose, weight)

    def make_controller(self, type, goal, weight):
        controller = Controller()
        controller.type = type
        controller.tip_link = self.tip
        controller.root_link = self.root

        controller.goal_pose = goal

        controller.p_gain = 3
        controller.enable_error_threshold = True
        controller.threshold_value = 0.2
        controller.weight = weight
        return controller

    def set_and_send_cartesian_goal(self, goal_pose):
        self.set_translation_goal(goal_pose)
        self.set_orientation_goal(goal_pose)
        self.send_cartesian_goal()

    def send_cartesian_goal(self):
        if self.enabled:
            goal = ControllerListGoal()
            goal.type = ControllerListGoal.STANDARD_CONTROLLER
            goal.controllers.append(self.translation_goal)
            goal.controllers.append(self.orientation_goal)
            self.send_goal(goal)

    def send_joint_goal(self, joint_state):
        if self.enabled:
            goal = ControllerListGoal()
            goal.type = ControllerListGoal.STANDARD_CONTROLLER

            controller = Controller()
            controller.type = Controller.JOINT
            controller.tip_link = self.tip
            controller.root_link = self.root

            controller.goal_state = joint_state

            controller.p_gain = 3
            controller.enable_error_threshold = False
            controller.threshold_value = 0.05
            controller.weight = 1.0
            goal.controllers.append(controller)

            self.send_goal(goal)

    def send_goal(self, goal):
        self.client.send_goal(goal)
        result = self.client.wait_for_result(rospy.Duration(self.move_time_limit))
        # print('finished in 10s?: {}'.format(result))

    def floor_detection_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.77669940148,
            -2.07846769483,
            1.79731767393,
            -2.48730626502,
            -1.37869861831,
            1.49620376209,
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

    def pre_baseboard_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.56896144549,
            -1.2928908507,
            1.59626483917,
            -2.61700326601,
            -1.54511577288,
            1.54054629803,
        ]
        self.send_joint_goal(joint_state)
