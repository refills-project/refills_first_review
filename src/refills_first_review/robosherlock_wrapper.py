from __future__ import print_function, division
import numpy as np

import rospy

from refills_first_review.barcode_detection import BarcodeDetector
from refills_first_review.baseboard_detection import BaseboardDetector
from refills_first_review.separator_detection import SeparatorClustering

FLOORS = {
    'shelf0': [0.15, 0.55, 0.88, 1.17, 1.43],
    'shelf1': [0.15, 0.38, 0.59, 1.11, 1.42],
    'shelf2': [0.15, 0.47, 0.76, 1.06, 1.39],
    'shelf3': [0.15, 0.43, 0.68, 0.93, 1.18, 1.43],
}


class RoboSherlock(object):
    def __init__(self):
        # TODO use paramserver [low]
        # TODO implement all the things [high]
        self.separator_detection = SeparatorClustering()
        self.baseboard_detection = BaseboardDetector()
        self.barcode_detection = BarcodeDetector()
        rospy.logwarn('robosherlock not fully integrated')

    def start_separator_detection(self, shelf_id, floor_id):
        # TODO
        self.separator_detection.start_listening(shelf_id, floor_id)

    def stop_separator_detection(self):
        # TODO
        return self.separator_detection.stop_listening()

    def start_baseboard_detection(self):
        # TODO
        self.baseboard_detection.start_listening()

    def stop_baseboard_detection(self):
        # TODO
        return self.baseboard_detection.stop_listening()

    def start_barcode_detection(self):
        # TODO
        # self.barcode_detection.start_listening()
        pass

    def stop_barcode_detection(self):
        # TODO
        # return self.barcode_detection.stop_listening()
        return {}

    def detect_floors(self, shelf_id):
        # TODO
        rospy.loginfo('detecting floors for shelf {}'.format(shelf_id))
        rospy.logwarn('floor detection not implemented')
        return FLOORS[shelf_id]

    def count(self):
        # TODO
        return int(np.random.random()*4)