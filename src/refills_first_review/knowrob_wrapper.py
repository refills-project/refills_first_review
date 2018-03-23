import json
from collections import OrderedDict, defaultdict

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from json_prolog import json_prolog
from refills_first_review.tfwrapper import TfWrapper

MAP = 'map'
DM_MARKET = 'dm_market'
SHELF_FRAME = 'DMShelfFrameFrontStore'
SHELF_FLOOR = 'DMShelfLayer4TilesFront'
SHELF_FLOOR_MOUNTING = 'DMShelfLayerMountingFront'
SEPARATOR = 'DMShelfSeparator4Tiles'
BARCODE = 'DMShelfLabel'
MOUNTING_BAR = 'DMShelfMountingBar'


class KnowRob(object):
    def __init__(self):
        # TODO implement all the things [high]
        # TODO use paramserver [low]
        self.tf = TfWrapper()
        self.floors = {}
        self.shelf_to_floor = defaultdict(list)  # TODO replace with prolog query
        self.shelves = {}
        self.shelf_ids = []
        self.separators = {}
        rospy.wait_for_service('/json_prolog/simple_query', 1)
        self.prolog = json_prolog.Prolog()

    def prolog_query(self, q):
        print(q)
        query = self.prolog.query(q)
        solutions = [x if x != {} else True for x in query.solutions()]
        if len(solutions) > 1:
            rospy.logwarn('{} returned more than one result'.format(q))
        elif len(solutions) == 0:
            rospy.logwarn('{} returned nothing'.format(q))
        query.finish()
        return solutions

    def remove_http_shit(self, s):
        return s.split('#')[-1].split('\'')[0]

    def load_barcode_to_mesh_map(self):
        self.barcode_to_mesh = json.load(open('../../data/barcode_to_mesh.json'))

    def pose_to_prolog(self, pose_stamped):
        return '[\'{}\', _, [{},{},{}], [{},{},{},{}]]'.format(pose_stamped.header.frame_id,
                                                               pose_stamped.pose.position.x,
                                                               pose_stamped.pose.position.y,
                                                               pose_stamped.pose.position.z,
                                                               pose_stamped.pose.orientation.x,
                                                               pose_stamped.pose.orientation.y,
                                                               pose_stamped.pose.orientation.z,
                                                               pose_stamped.pose.orientation.w)

    # shelves
    def add_shelves(self, shelves):
        # TODO failure handling
        self.shelves = OrderedDict()
        for name, pose in shelves.items():
            q = 'belief_perceived_at({}:\'{}\',{}, 0.01, R)'.format(DM_MARKET,
                                                                    SHELF_FRAME,
                                                                    self.pose_to_prolog(pose))
            solutions = self.prolog_query(q)
            # object_id = solutions[0]['R'].replace('\'','')
            # self.shelves[object_id] = pose
        return True

    def get_objects(self, type):
        # TODO rdfs or owl?
        # TODO failure handling
        objects = OrderedDict()
        q = 'rdfs_individual_of(R, {}:\'{}\').'.format(DM_MARKET, type)
        solutions = self.prolog_query(q)
        for solution in solutions:
            object_id = solution['R'].replace('\'', '')
            pose_q = 'belief_at(\'{}\', R).'.format(object_id)
            believed_pose = self.prolog_query(pose_q)[0]['R']
            ros_pose = PoseStamped()
            ros_pose.header.frame_id = believed_pose[0]
            ros_pose.pose.position = Point(*believed_pose[2])
            ros_pose.pose.orientation = Quaternion(*believed_pose[3])
            objects[object_id] = ros_pose
        return objects

    def get_shelves(self):
        return self.get_objects(SHELF_FRAME)

    def get_object_frame_id(self, object_id):
        q = 'object_frame_name(\'{}\', R).'.format(object_id)
        return self.prolog_query(q)[0]['R'].replace('\'', '')

    # floor
    def add_shelf_floors(self, shelf_id, floors):
        frame_id = self.get_object_frame_id(shelf_id)
        for position in floors:
            type = SHELF_FLOOR if position[1] < 0.13 else SHELF_FLOOR_MOUNTING
            p = PoseStamped()
            p.header.frame_id = frame_id
            p.pose.position = Point(*position)
            p.pose.orientation.w = 1
            q = 'belief_perceived_at({}:\'{}\',{}, 0.01, R)'.format(DM_MARKET, type, self.pose_to_prolog(p))
            solutions = self.prolog_query(q)
            object_id = solutions[0]['R'].replace('\'', '')
            self.shelf_to_floor[shelf_id].append(object_id)
        return True

    def get_floor_ids(self, shelf_id):
        self.floors = self.get_objects(SHELF_FLOOR)
        self.floors.update(self.get_objects(SHELF_FLOOR_MOUNTING))
        floors = []
        frame_id = self.get_object_frame_id(shelf_id)
        for floor, pose in self.floors.items():
            if floor in self.shelf_to_floor[shelf_id]:
                floors.append((floor, self.tf.transform_pose(frame_id, pose)))
        floors = list(sorted(floors, key=lambda x: x[1].pose.position.z))
        self.floors = OrderedDict(floors)
        return self.floors

    def get_floor_width(self):
        # TODO
        return 1.0

    def get_floor_position(self, floor_id):
        return self.floors[floor_id]

    def is_floor_too_high(self, floor_id):
        return self.get_floor_position(floor_id).pose.position.z > 1.2

    def is_bottom_floor(self, floor_id):
        return self.get_floor_position(floor_id).pose.position.z < 0.16

    def is_hanging_foor(self, floor_id):
        q = 'rdfs_individual_of(\'{}\', {}:\'{}\')'.format(floor_id, DM_MARKET, SHELF_FLOOR_MOUNTING)
        solutions = self.prolog_query(q)
        return len(solutions) > 0

    def is_normal_floor(self, floor_id):
        return not self.is_bottom_floor(floor_id) and not self.is_hanging_foor(floor_id)

    def add_separators(self, shelf_id, floor_id, separators):
        for p in separators:
            q = 'belief_perceived_at({}:\'{}\',{}, 0.01, R)'.format(DM_MARKET, SEPARATOR, self.pose_to_prolog(p))
            solutions = self.prolog_query(q)
        # self.separators[shelf_id, floor_id] = separators
        return True

    def add_barcodes(self, barcodes):
        for barcode, p in barcodes.items():
            q = 'belief_perceived_at({}:\'{}\',{}, 0.01, R)'.format(DM_MARKET, BARCODE, self.pose_to_prolog(p))
            solutions = self.prolog_query(q)
        pass

    def get_facings(self, shelf_id, floor_id):
        # TODO ask knowrob
        return []
        # separators = []
        # frame_id = self.get_object_frame_id(shelf_id)
        # for separator in self.separators[shelf_id, floor_id]:
        #     separator.header.stamp = rospy.Time()
        #     separator_map = self.tf.transform_pose(frame_id, separator)
        #     separators.append(separator_map.pose.position.x)
        # facings = []
        # separators = list(sorted(separators))
        # for i, facing in enumerate(sorted(separators)[:-1]):
        #     facings.append((facing + separators[i + 1]) / 2)
        # return facings
