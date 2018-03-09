import json
import rospy

from refills_first_review.tfwrapper import TfWrapper


class KnowRob(object):
    def __init__(self):
        # TODO setup knowrob connection [high]
        # TODO implement all the things [high]
        # TODO use paramserver [low]
        self.tf = TfWrapper()
        self.floors = {}
        self.shelves = {}
        self.separators = {}
        rospy.logwarn('knowrob not fully integrated')

    def load_barcode_to_mesh_map(self):
        self.barcode_to_mesh = json.load(open('../../data/barcode_to_mesh.json'))

    # shelves
    def add_shelves(self, shelves):
        # TODO
        self.shelves = shelves
        for name, pose in self.shelves.items():
            self.tf.add_frame_from_pose(name, pose)
        self.tf.start_frame_broadcasting()
        return True

    def get_shelves(self):
        # TODO
        return self.shelves

    def get_shelf_frame_id(self, shelf_id):
        # TODO
        return shelf_id

    # floor
    def add_shelf_floors(self, shelf_id, floors):
        # TODO
        self.floors[shelf_id] = floors
        return True

    def get_floor_ids(self, shelf_id):
        # TODO
        return list(range(len(self.floors[shelf_id])))

    def get_floor_width(self):
        # TODO
        return 1.0

    def get_floor_height(self, shelf_id, floor_id):
        # TODO
        return self.floors[shelf_id][floor_id]

    def is_floor_too_high(self, shelf_id, floor_id):
        # TODO
        return self.get_floor_height(shelf_id, floor_id) > 1.2

    def is_bottom_floor(self, shelf_id, floor_id):
        return floor_id == 0

    def is_hanging_foor(self, shelf_id, floor_id):
        return shelf_id == 'shelf1' and floor_id in [3, 4, 5]

    def is_normal_floor(self, shelf_id, floor_id):
        return not self.is_bottom_floor(shelf_id, floor_id) and not self.is_hanging_foor(shelf_id, floor_id)

    def add_separators(self, shelf_id, floor_id, separators):
        self.separators[shelf_id, floor_id] = separators
        return True

    def add_barcodes(self, barcodes):
        # TODO
        pass

    def get_facings(self, shelf_id, floor_id):
        separators = []
        for separator in self.separators[shelf_id, floor_id]:
            separator.header.stamp = rospy.Time()
            separator_map = self.tf.transform_pose(self.get_shelf_frame_id(shelf_id), separator)
            separators.append(separator_map.pose.position.y)
        facings = []
        separators = list(sorted(separators))
        for i, facing in enumerate(sorted(separators)[:-1]):
            facings.append((facing + separators[i+1])/2)
        return facings

    def get_object_mesh(self, barcode):
        pass
