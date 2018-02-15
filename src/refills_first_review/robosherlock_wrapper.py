from __future__ import print_function, division
import numpy as np

import rospy

FLOORS = {
    'shelf0': [0.1, 0.4, 0.75, 1.05, 1.25],
    'shelf1': [0.1, 0.25, 0.45, 0.95, 1.25],
    'shelf2': [0.1, 0.3, 0.6, 0.9, 1.25],
    'shelf3': [0.1, 0.25, 0.5, 0.75, 1.0, 1.2],
}


class RoboSherlock(object):
    def __init__(self):
        pass

    def detect_floors(self, shelf_id):
        # TODO
        rospy.loginfo('detecting floors for shelf {}'.format(shelf_id))
        rospy.logwarn('floor detection not implemented')
        return FLOORS[shelf_id]

    def count(self):
        return int(np.random.random()*4)