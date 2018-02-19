from __future__ import print_function, division
import numpy as np

import rospy

FLOORS = {
    'shelf0': [0.15, 0.55, 0.88, 1.17, 1.43],
    'shelf1': [0.15, 0.38, 0.59, 1.11, 1.42],
    'shelf2': [0.15, 0.47, 0.76, 1.06, 1.39],
    'shelf3': [0.15, 0.43, 0.68, 0.93, 1.18, 1.43],
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