import rospy


class KnowRob(object):
    def __init__(self):
        # TODO setup knowrob connection [high]
        # TODO implement all the things [high]
        # TODO use paramserver [low]
        self.floors = {}
        self.shelves = {}
        rospy.logwarn('knowrob not fully integrated')

    # shelves
    def add_shelves(self, shelves):
        # TODO
        self.shelves = shelves
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

    def add_separators(self, separators):
        # TODO
        return True

    def add_barcodes(self, barcodes):
        # TODO
        pass

    def get_facings(self, shelf_id, floor_id):
        # TODO
        return [.2, .4, .6, .8]
