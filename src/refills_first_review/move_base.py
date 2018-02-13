from multiprocessing import TimeoutError

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header


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
            # rospy.loginfo('arrived at base goal {}'.format(result))
            return result

    def move_relative(self, position=(0,0,0), orientation=(0,0,0,1)):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'base_footprint'
        shelf.header = header
        shelf.pose.position = Point(*position)
        shelf.pose.orientation = Quaternion(*orientation)
        self.move_absolute(shelf)

    # def goto_shelf1(self):
    #     # self.client.cancel_all_goals()
    #     shelf = PoseStamped()
    #     header = Header()
    #     header.frame_id = 'map'
    #     shelf.header = header
    #     shelf.pose.position = Point(-0.0, self.dist_to_shelfs, 0.0)
    #     shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    #     self.move_absolute(shelf)
    #
    # def goto_shelf2(self):
    #     shelf = PoseStamped()
    #     header = Header()
    #     header.frame_id = 'map'
    #     shelf.header = header
    #     shelf.pose.position = Point(-1.0, self.dist_to_shelfs, 0.0)
    #     shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    #     self.move_absolute(shelf)
    #
    # def goto_shelf3(self):
    #     shelf = PoseStamped()
    #     header = Header()
    #     header.frame_id = 'map'
    #     shelf.header = header
    #     shelf.pose.position = Point(-2.0, self.dist_to_shelfs, 0.000)
    #     shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    #     # shelf.pose.orientation = Quaternion(*quaternion_from_euler(0,0,pi*.98))
    #     self.move_absolute(shelf)
    #
    # def goto_shelf4(self):
    #     shelf = PoseStamped()
    #     header = Header()
    #     header.frame_id = 'map'
    #     shelf.header = header
    #     shelf.pose.position = Point(-2.9, self.dist_to_shelfs, 0.000)
    #     shelf.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    #     # shelf.pose.orientation = Quaternion(*quaternion_from_euler(0,0,pi*.99))
    #     self.move_absolute(shelf)

