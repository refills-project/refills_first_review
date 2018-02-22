import rospy
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener


class TfWrapper(object):
    def __init__(self, buffer_size=2):
        self.tfBuffer = Buffer(rospy.Duration(buffer_size))
        self.tf_listener = TransformListener(self.tfBuffer)
        rospy.sleep(0.1)

    def transform_pose(self, target_frame, pose):
        try:
            transform = self.tfBuffer.lookup_transform(target_frame,
                                                       pose.header.frame_id,  # source frame
                                                       pose.header.stamp,
                                                       rospy.Duration(1.0))
            new_pose = do_transform_pose(pose, transform)
            return new_pose
        except ExtrapolationException as e:
            rospy.logwarn(e)

    def lookup_transform(self, target_frame, source_frame):
        p = PoseStamped()
        p.header.frame_id = source_frame
        p.pose.orientation.w = 1.0
        return self.transform_pose(target_frame, p)