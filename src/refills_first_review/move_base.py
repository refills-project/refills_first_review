from multiprocessing import TimeoutError

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from tf.transformations import quaternion_about_axis


class MoveBase(object):
    def __init__(self, move_base_action_name='nav_pcontroller/move_base', enabled=False):
        self.enabled = enabled
        self.client = actionlib.SimpleActionClient(move_base_action_name, MoveBaseAction)
        rospy.loginfo('connecting to {} ...'.format(move_base_action_name))
        self.client.wait_for_server()
        rospy.loginfo('connected to {}'.format(move_base_action_name))
        self.goal_pub = rospy.Publisher('move_base_goal', PoseStamped, queue_size=10)
        rospy.sleep(0.5)
        self.timeout = 30
        self.dist_to_shelfs = 1.4

    def move_absolute(self, target_pose):
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
                # self.STOP()
                raise TimeoutError()
            return result

    def move_absolute_xyz(self, frame_id, x, y, z):
        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.orientation = Quaternion(*quaternion_about_axis(z, [0,0,1]))
        return self.move_absolute(target_pose)

    def move_relative(self, position=(0,0,0), orientation=(0,0,0,1)):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'base_footprint'
        shelf.header = header
        shelf.pose.position = Point(*position)
        shelf.pose.orientation = Quaternion(*orientation)
        self.move_absolute(shelf)

    def STOP(self):
        self.client.cancel_goal()
        self.move_relative()
