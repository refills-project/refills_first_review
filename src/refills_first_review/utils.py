import PyKDL
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import numpy as np

from tf.transformations import quaternion_from_matrix


def posestamped_to_kdl(posestamped):
    """Convert a geometry_msgs Transform message to a PyKDL Frame.

    :param posestamped: The Transform message to convert.
    :type posestamped: PoseStamped
    :return: The converted PyKDL frame.
    :rtype: PyKDL.Frame
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(posestamped.pose.orientation.x,
                                                 posestamped.pose.orientation.y,
                                                 posestamped.pose.orientation.z,
                                                 posestamped.pose.orientation.w),
                       PyKDL.Vector(posestamped.pose.position.x,
                                    posestamped.pose.position.y,
                                    posestamped.pose.position.z))


def kdl_to_pose(frame):
    """
    :type frame: PyKDL.Frame
    :rtype: Pose
    """
    p = Pose()
    p.position.x = frame.p[0]
    p.position.y = frame.p[1]
    p.position.z = frame.p[2]
    m = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2], 0],
                  [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2], 0],
                  [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2], 0],
                  [0, 0, 0, 1]])
    p.orientation = Quaternion(*quaternion_from_matrix(m))
    return p
