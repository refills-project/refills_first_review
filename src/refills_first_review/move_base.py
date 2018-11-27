from multiprocessing import TimeoutError
import numpy as np
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from tf.transformations import quaternion_about_axis
import PyKDL
from tf2_kdl import transform_to_kdl

from refills_first_review.tfwrapper import lookup_transform, lookup_transform2, transform_pose
from refills_first_review.utils import posestamped_to_kdl, kdl_to_pose


class MoveBase(object):
    def __init__(self, move_base_action_name='nav_pcontroller/move_base', enabled=True, knowrob=None):
        self.enabled = enabled
        self.knowrob = knowrob
        self.client = actionlib.SimpleActionClient(move_base_action_name, MoveBaseAction)
        rospy.loginfo('connecting to {} ...'.format(move_base_action_name))
        self.client.wait_for_server()
        rospy.loginfo('connected to {}'.format(move_base_action_name))
        self.goal_pub = rospy.Publisher('move_base_goal', PoseStamped, queue_size=10)
        self.controlled_link = 'base_footprint'
        rospy.sleep(0.5)
        self.timeout = 50
        self.dist_to_shelfs = 1.4

    def move_absolute(self, target_pose, retry=True):
        if self.enabled:
            self.goal_pub.publish(target_pose)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            if self.knowrob is not None:
                self.knowrob.start_base_movement(self.knowrob.pose_to_prolog(target_pose))
            while True:
                self.client.send_goal(goal)
                wait_result = self.client.wait_for_result(rospy.Duration(self.timeout))
                result = self.client.get_result()
                state = self.client.get_state()
                if wait_result and state == GoalStatus.SUCCEEDED:
                    break
                if retry:
                    cmd = raw_input('base movement did not finish in time, retry? [y/n]')
                    retry = cmd == 'y'
                if not retry:
                    print('movement did not finish in time')
                    if self.knowrob is not None:
                        self.knowrob.finish_action()
                    raise TimeoutError()
            if self.knowrob is not None:
                self.knowrob.finish_action()
            return result

    def move_absolute_link(self, goal_pose, link='camera_link'):
        """
        :type goal_pose: PoseStamped
        :return:
        """
        t_base___camera = transform_to_kdl(lookup_transform2(self.controlled_link, link))
        t_base___camera.M = PyKDL.Rotation()
        t_base___camera.p[2] = 0
        goal_pose = transform_pose('map', goal_pose)
        t_map___goal_camera = posestamped_to_kdl(goal_pose)
        t_map___goal_camera.p[2] = 0
        t_map___goal_base = t_map___goal_camera * t_base___camera.Inverse()
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose = kdl_to_pose(t_map___goal_base)
        self.move_absolute(base_goal)

    def move_absolute_xyz(self, frame_id, x, y, z, retry=True):
        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.orientation = Quaternion(*quaternion_about_axis(z, [0, 0, 1]))
        return self.move_absolute(target_pose, retry)

    def move_relative(self, position=(0, 0, 0), orientation=(0, 0, 0, 1), retry=True):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'base_footprint'
        shelf.header = header
        shelf.pose.position = Point(*position)
        shelf.pose.orientation = Quaternion(*orientation)
        self.move_absolute(shelf, retry)

    def STOP(self):
        self.client.cancel_goal()
        self.move_relative(retry=False)

    def get_c(self):
        pass

    def is_stuff_close(self, threshold=3):
        # TODO implement, maybe move somewhere else [low]
        rospy.logwarn('closest point not implemented')
        return False

if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    g = MoveBase()
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = 1
    goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi/2, [0,0,1]))
    g.move_absolute_link(goal)