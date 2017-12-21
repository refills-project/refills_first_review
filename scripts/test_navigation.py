#!/usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import tf2_ros

from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from move_base_msgs.msg._MoveBaseAction import MoveBaseAction
from move_base_msgs.msg._MoveBaseGoal import MoveBaseGoal
from std_msgs.msg._Header import Header
from numpy import pi
import numpy as np

from tf.transformations import rotation_from_matrix, quaternion_matrix


class MoveBase(object):
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.client = actionlib.SimpleActionClient('nav_pcontroller/move_base', MoveBaseAction)
        print('connecting ...')
        self.client.wait_for_server()
        print('connected')
        self.goal_pub = rospy.Publisher('move_base_goal', PoseStamped, queue_size=10)
        rospy.sleep(0.5)
        self.dist_to_shelfs = 1.4

    def __call__(self, target_pose):
        # self.client.cancel_all_goals()
        if self.enabled:
            self.goal_pub.publish(target_pose)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(120))
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

    def move_to(self, target_pose):
        self(target_pose)

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

def goal_diff(goal, transform):
    t1 = [goal.pose.position.x,
          goal.pose.position.y,
          goal.pose.position.z,]
    t2 = [transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z,]
    rospy.loginfo('translation diff = {}m'.format(np.linalg.norm(np.array(t1)-np.array(t2))))
    q1 = [goal.pose.orientation.x,
          goal.pose.orientation.y,
          goal.pose.orientation.z,
          goal.pose.orientation.w,]
    q2 = [transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w,]
    q1 = np.array(q1)
    q2 = np.array(q2)
    angle1 = rotation_from_matrix(quaternion_matrix(q1))[0]
    angle2 = rotation_from_matrix(quaternion_matrix(q2))[0]
    rospy.loginfo('rotation diff = {} deg'.format(abs(angle1 - angle2)))

if __name__ == '__main__':
    rospy.init_node('brain')
    move_base = MoveBase(True)
    move_base.client.cancel_all_goals()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    goals = []

    while not rospy.is_shutdown():
        cmd = raw_input('[add|clear|replay|q] - ')
        if cmd == 'add':
            transform = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = transform.header.frame_id
            goal_pose.pose.position.x = transform.transform.translation.x
            goal_pose.pose.position.y = transform.transform.translation.y
            goal_pose.pose.position.z = transform.transform.translation.z
            goal_pose.pose.orientation = transform.transform.rotation
            goals.append(goal_pose)
        elif cmd == 'clear':
            goals = []
        elif cmd == 'replay':
            for goal in goals:
                rospy.loginfo('moving to: \n{}'.format(goal))
                move_base.move_to(goal)
                transform = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
                goal_diff(goal, transform)
                cmd = raw_input('[enter] to continue')
            rospy.loginfo('done')
        elif cmd == 'q':
            rospy.loginfo('quit')
            break


