class KnowRob(object):
    def __init__(self):
        # TODO setup knowrob connection
        self.floors = {}
        pass

    def add_shelf_floors(self, shelf_id, floors):
        # TODO
        self.floors[shelf_id] = floors
        return True

    def add_separators(self, separators):
        # TODO
        return True

    def get_floor_width(self):
        return 1.0

    def get_shelf_frame_id(self, shelf_id):
        return shelf_id

    def get_facings(self, shelf_id, floor_id):
        return [.2,.4,.6,.8]

    def get_floor_height(self, shelf_id, floor_id):
        return self.floors[shelf_id][floor_id]

    def is_floor_too_high(self, shelf_id, floor_id):
        return self.get_floor_height(shelf_id, floor_id) > 1.2
