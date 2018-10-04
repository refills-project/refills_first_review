import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Quaternion, Point, PoseStamped, QuaternionStamped, PointStamped
from giskard_msgs.msg import Controller, ControllerListGoal, ControllerListAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class GiskardWrapper(object):
    def __init__(self, giskard_action_name='/qp_controller/command', enabled=True, knowrob=None):
        self.move_time_limit = 25
        self.enabled = enabled
        self.knowrob = knowrob
        self.client = SimpleActionClient(giskard_action_name, ControllerListAction)
        rospy.loginfo('connecting to {}'.format(giskard_action_name))
        self.client.wait_for_server()
        rospy.loginfo('connected to {}'.format(giskard_action_name))
        self.tip = 'camera_link'
        self.root = 'base_link'

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
        if self.knowrob is not None:
            if goal.controllers[0].type == Controller.JOINT:
                self.knowrob.start_head_movement()
            else:
                self.knowrob.start_head_movement(self.knowrob.pose_to_prolog(goal.controllers[0].goal_pose))
                # self.knowrob.start_head_movement()
        self.client.send_goal(goal)
        result = self.client.wait_for_result(rospy.Duration(self.move_time_limit))
        if self.knowrob is not None:
            self.knowrob.finish_action()
        if not result:
            raise Exception('arm movement failed')
        # print('finished in 10s?: {}'.format(result))

    def floor_detection_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.63407260576,
            -1.4751423041,
            0.677300930023,
            -2.12363607088,
            -1.50967580477,
            1.55717146397,
        ]
        self.send_joint_goal(joint_state)

    def drive_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.54838782946,
            -2.51830751101,
            1.37984895706,
            -2.07125074068,
            -1.59281522432,
            1.56871032715,

        ]
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


if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    ma = GiskardWrapper()
    # ma.floor_detection_pose2()
    ma.set_orientation_goal(QuaternionStamped(Header(0, rospy.Time(), ma.root),
                                              Quaternion(0., -0.70660617, 0.70760703, 0.)))
    ma.send_cartesian_goal()
