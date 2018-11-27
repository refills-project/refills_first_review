
import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Quaternion, Point, PoseStamped, QuaternionStamped, PointStamped
from giskard_msgs.msg import Controller, ControllerListGoal, ControllerListAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf.transformations import quaternion_about_axis, quaternion_from_matrix
import numpy as np

class GiskardWrapper(object):
    def __init__(self, giskard_action_name='/qp_controller/command', enabled=True, knowrob=None):
        self.move_time_limit = 120
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

    def set_translation_goal(self, translation, weight=1.0, threshold_value=0.2):
        goal_pose = PoseStamped()
        if isinstance(translation, PointStamped):
            goal_pose.header = translation.header
            goal_pose.pose.position = translation.point
        else:
            goal_pose = translation
        self.translation_goal = self.make_controller(Controller.TRANSLATION_3D, goal_pose, weight, threshold_value)

    def set_orientation_goal(self, orientation, weight=1.0, threshold_value=0.2):
        goal_pose = PoseStamped()
        if isinstance(orientation, QuaternionStamped):
            goal_pose.header = orientation.header
            goal_pose.pose.orientation = orientation.quaternion
        else:
            goal_pose = orientation
        self.orientation_goal = self.make_controller(Controller.ROTATION_3D, goal_pose, weight, threshold_value)

    def rotate_x(self, angle):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.tip
        goal_pose.pose.orientation = Quaternion(*quaternion_about_axis(angle, [1,0,0]))
        self.set_and_send_cartesian_goal(goal_pose)

    def make_controller(self, type, goal, weight, threshold_value=0.2):
        controller = Controller()
        controller.type = type
        controller.tip_link = self.tip
        controller.root_link = self.root

        controller.goal_pose = goal

        controller.p_gain = 3
        controller.enable_error_threshold = True
        controller.threshold_value = threshold_value
        controller.weight = weight
        return controller

    def set_and_send_cartesian_goal(self, goal_pose, threshold_value=0.2):
        self.set_translation_goal(goal_pose, threshold_value=threshold_value)
        self.set_orientation_goal(goal_pose, threshold_value=threshold_value)
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
            0.0,
            -1.768,
            -0.51,
            -2.396,
            0.2181,
            -3.191,
        ]
        self.send_joint_goal(joint_state)
    # def floor_detection_pose(self):
    #     joint_state = JointState()
    #     joint_state.name = self.joint_names
    #     joint_state.position = [
    #         -np.pi,
    #         -1.52270842207,
    #         0.541624814496,
    #         -1.0,
    #         -0.23,
    #         0.4,
    #     ]
    #     self.send_joint_goal(joint_state)



    def drive_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            0,
            -0.88,
            -1.351,
            -2.4346,
            0.21823,
            -3.199,
        ]
        self.send_joint_goal(joint_state)

    def pre_baseboard_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            0.0,
            -1.6460,
            -2.171,
            -0.85549,
            0.2181,
            -3.19172,
        ]
        self.send_joint_goal(joint_state)


if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    g = GiskardWrapper()
    # qs = QuaternionStamped()
    # qs.quaternion = Quaternion(*quaternion_from_matrix(np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]])))
    # qs.header.frame_id = 'base_footprint'
    # ps = PointStamped()
    # ps.header.frame_id = 'camera_link'
    # g.set_orientation_goal(qs)
    # g.set_translation_goal(ps)
    # g.send_cartesian_goal()
