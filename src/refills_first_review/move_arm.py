import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Quaternion, Point, PoseStamped, QuaternionStamped, PointStamped
from giskard_msgs.msg import Controller, ControllerListGoal, ControllerListAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf.transformations import quaternion_about_axis

from giskardpy import python_interface


class GiskardWrapper(object):
    def __init__(self, enabled=True, knowrob=None):
        self.move_time_limit = 25
        self.enabled = enabled
        self.knowrob = knowrob
        self.giskard = python_interface.GiskardWrapper()
        self.tip = 'camera_link'
        self.root = 'base_link'
        self.trans_p_gain = 1
        self.rot_p_gain = 1
        self.trans_max_speed = 0.1
        self.rot_max_speed = 1

        # TODO get this from param server of topic
        self.joint_names = ['ur5_shoulder_pan_joint',
                            'ur5_shoulder_lift_joint',
                            'ur5_elbow_joint',
                            'ur5_wrist_1_joint',
                            'ur5_wrist_2_joint',
                            'ur5_wrist_3_joint', ]

    def set_translation_goal(self, translation):
        goal_pose = PoseStamped()
        if isinstance(translation, PointStamped):
            goal_pose.header = translation.header
            goal_pose.pose.position = translation.point
        else:
            goal_pose = translation
        self.giskard.set_tranlation_goal(self.root, self.tip, goal_pose, self.trans_p_gain, self.trans_max_speed)

    def set_orientation_goal(self, orientation):
        goal_pose = PoseStamped()
        if isinstance(orientation, QuaternionStamped):
            goal_pose.header = orientation.header
            goal_pose.pose.orientation = orientation.quaternion
        else:
            goal_pose = orientation
        self.giskard.set_rotation_goal(self.root, self.tip, goal_pose, self.rot_p_gain, self.rot_max_speed)

    def set_and_send_cartesian_goal(self, goal_pose):
        self.set_translation_goal(goal_pose)
        self.set_orientation_goal(goal_pose)
        self.giskard.disable_self_collision()
        self.giskard.plan_and_execute()

    def send_cartesian_goal(self):
        if self.enabled:
            self.giskard.disable_self_collision()
            self.giskard.plan_and_execute()

    def send_joint_goal(self, joint_state):
        if self.enabled:
            self.giskard.set_joint_goal(joint_state)
            self.giskard.disable_self_collision()
            self.giskard.plan_and_execute()

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
        # joint_state.name = self.joint_names + ['odom_x_joint', 'odom_y_joint', 'odom_z_joint']
        joint_state.position = [
            -1.54838782946,
            -2.51830751101,
            1.37984895706,
            -2.07125074068,
            -1.59281522432,
            1.56871032715,
            # 0,
            # 0,
            # 0,
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
    ma.drive_pose()
    ma.floor_detection_pose2()
