import json
from collections import OrderedDict, defaultdict
from rospkg import RosPack

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from json_prolog import json_prolog
from refills_first_review.tfwrapper import TfWrapper

MAP = 'map'
SHOP = 'shop'
SHELF_FLOOR = '{}:\'ShelfLayer\''.format(SHOP)
DM_MARKET = 'dmshop'
SHELF_SYSTEM = '{}:\'DMShelfSystem\''.format(DM_MARKET)
SHELF_METER = '{}:\'DMShelfFrameFrontStore\''.format(DM_MARKET)
SHELF_FLOOR_STANDING = '{}:\'DMShelfLayer4TilesFront\''.format(DM_MARKET)
SHELF_FLOOR_MOUNTING = '{}:\'DMShelfLayerMountingFront\''.format(DM_MARKET)
SEPARATOR = '{}:\'DMShelfSeparator4Tiles\''.format(DM_MARKET)
BARCODE = '{}:\'DMShelfLabel\''.format(DM_MARKET)
MOUNTING_BAR = '{}:\'DMShelfMountingBar\''.format(DM_MARKET)


class ActionGraph(object):
    def __init__(self, knowrob, action_type=None, parent_node=None, previous_node=None, msg=''):
        self.msg = msg
        self.action_type = action_type
        self.knowrob = knowrob
        self.previous_node = previous_node
        self.parent_node = parent_node
        self.last_sub_action = None
        self.id = self.start(action_type)

    def start(self, action_type):
        if self.parent_node is None:
            q = 'cram_start_action(\'{}\', \'{}\', {}, _, R)'.format(action_type, self.msg, rospy.get_rostime())
        elif self.previous_node is not None:
            q = 'cram_start_action(\'{}\', \'{}\', {}, \'{}\', \'{}\', R)'.format(action_type, self.msg, rospy.get_rostime(),
                                                                           self.previous_node.id, self.parent_node.id)
        else:
            q = 'cram_start_action(\'{}\', \'{}\', {}, _, \'{}\', R)'.format(action_type, self.msg, rospy.get_rostime(),
                                                                          self.parent_node.id)
        return self.knowrob.prolog_query(q)[0]['R'].replace('\'', '')

    def finish(self):
        q = 'cram_finish_action(\'{}\', {})'.format(self.id, rospy.get_rostime())
        self.knowrob.prolog_query(q)
        return self.parent_node

    def add_sub_action(self, action_type, msg=''):
        self.last_sub_action = ActionGraph(self.knowrob, action_type, previous_node=self.last_sub_action,
                                           parent_node=self, msg=msg)
        return self.last_sub_action

    def __str__(self):
        return self.id.split('3')[-1]


class KnowRob(object):
    def __init__(self):
        # TODO implement all the things [high]
        # TODO use paramserver [low]
        self.floors = {}
        self.shelf_ids = []
        self.separators = {}
        self.action_graph = None
        self.tf = TfWrapper()
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

    def prolog_to_pose_msg(self, query_result):
        ros_pose = PoseStamped()
        ros_pose.header.frame_id = query_result[0]
        ros_pose.pose.position = Point(*query_result[2])
        ros_pose.pose.orientation = Quaternion(*query_result[3])
        return ros_pose

    def add_shelf_system(self):
        q = 'belief_new_object({}, R)'.format(SHELF_SYSTEM)
        shelf_system_id = self.prolog_query(q)[0]['R'].replace('\'', '')
        return shelf_system_id

    # shelves
    def add_shelves(self, shelf_system_id, shelves):
        # TODO failure handling
        for name, pose in shelves.items():
            q = 'belief_new_object({}, ID), ' \
                'rdf_assert(\'{}\', knowrob:properPhysicalParts, ID),' \
                'object_affordance_static_transform(ID, A, [_,_,T,R])'.format(SHELF_METER, shelf_system_id)
            solutions = self.prolog_query(q)[0]
            pose.pose.position.x -= solutions['T'][0]
            pose.pose.position.y -= solutions['T'][1]
            pose.pose.position.z -= solutions['T'][2]
            object_id = solutions['ID'].replace('\'', '')
            q = 'belief_at_update(\'{}\', {})'.format(object_id, self.pose_to_prolog(pose))
            solutions = self.prolog_query(q)

        return True

    def get_objects(self, type):
        # TODO failure handling
        objects = OrderedDict()
        q = 'rdfs_individual_of(R, {}).'.format(type)
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
        return self.get_objects(SHELF_METER)

    def get_perceived_frame_id(self, object_id):
        # TODO there has to be a better way
        q = 'object_affordance_static_transform(\'{}\', A, _), object_frame_name(A, R).'.format(object_id)
        return self.prolog_query(q)[0]['R'].replace('\'', '')

    def get_object_frame_id(self, object_id):
        q = 'object_frame_name(\'{}\', R).'.format(object_id)
        return self.prolog_query(q)[0]['R'].replace('\'', '')

    # floor
    def add_shelf_floors(self, shelf_id, floors):
        perceived_frame_id = self.get_perceived_frame_id(shelf_id)
        frame_id = self.get_object_frame_id(shelf_id)
        for position in floors:
            type = SHELF_FLOOR_STANDING if position[1] < 0.13 else SHELF_FLOOR_MOUNTING
            p = PoseStamped()
            p.header.frame_id = perceived_frame_id
            p.pose.position = Point(*position)
            p.pose.orientation.w = 1
            p = self.tf.transform_pose(frame_id, p)
            q = 'belief_new_object({}, FloorId), ' \
                'rdf_assert(\'{}\', knowrob:properPhysicalParts, FloorId),' \
                'object_affordance_static_transform(FloorId, _, [_,_,T,R])'.format(type, shelf_id)
            solutions = self.prolog_query(q)[0]
            p.pose.position.x -= solutions['T'][0]
            p.pose.position.y -= solutions['T'][1]
            p.pose.position.z -= solutions['T'][2]
            q = 'belief_at_update({}, {})'.format(solutions['FloorId'], self.pose_to_prolog(p))
            self.prolog_query(q)
        return True

    def get_floor_ids(self, shelf_id):
        q = 'rdf_has(\'{}\', knowrob:properPhysicalParts, R), ' \
            'rdfs_individual_of(R, {})'.format(shelf_id, SHELF_FLOOR)
        solutions = self.prolog_query(q)
        floors = []
        shelf_frame_id = self.get_perceived_frame_id(shelf_id)
        for solution in solutions:
            floor_id = solution['R'].replace('\'', '')
            floor_pose = self.tf.lookup_transform(shelf_frame_id, self.get_perceived_frame_id(floor_id))
            floors.append((floor_id, floor_pose))
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
        q = 'rdfs_individual_of(\'{}\', {})'.format(floor_id, SHELF_FLOOR_MOUNTING)
        solutions = self.prolog_query(q)
        return len(solutions) > 0

    def is_normal_floor(self, floor_id):
        return not self.is_bottom_floor(floor_id) and not self.is_hanging_foor(floor_id)

    def add_separators(self, floor_id, separators):
        for p in separators:
            q = 'shelf_layer_spawn(\'{}\', {}, {}, _)'.format(floor_id, SEPARATOR, p.pose.position.x)
            solutions = self.prolog_query(q)
        return True

    def add_barcodes(self, floor_id, barcodes):
        for barcode, p in barcodes.items():
            q = 'shelf_layer_spawn_label(\'{}\', {}, dan(\'{}\'), {}, _)'.format(floor_id, BARCODE,
                                                                                 barcode, p.pose.position.x)
            solutions = self.prolog_query(q)

    def add_separators_and_barcodes(self, floor_id, separators, barcodes):
        separator_q = ','.join(['shelf_layer_spawn(\'{}\', {}, {}, _)'.format(floor_id, SEPARATOR, p.pose.position.x)
                                for p in separators])

        barcode_q = ','.join(['shelf_layer_spawn_label(\'{}\', {}, dan(\'{}\'), {}, _)'.format(floor_id, BARCODE,
                                                                                               barcode,
                                                                                               p.pose.position.x)
                              for barcode, p in barcodes.items()])
        self.prolog_query('{},{}'.format(separator_q, barcode_q))

    def get_facings(self, floor_id):
        q = 'findall(F, shelf_facing(\'{}\', F), R).'.format(floor_id)
        solutions = self.prolog_query(q)[0]
        facings = []
        for facing_id in solutions['R']:
            facing_pose = self.tf.lookup_transform(self.get_perceived_frame_id(floor_id),
                                                   self.get_object_frame_id(facing_id))
            facings.append(facing_pose)
        return facings

    def save_beliefstate(self):
        path = '{}/data/beliefstate.owl'.format(RosPack().get_path('refills_first_review'))
        q = 'rdf_save(\'{}\', belief_state)'.format(path)
        self.prolog_query(q)

    def save_action_graph(self):
        path = '{}/data/action_graph.owl'.format(RosPack().get_path('refills_first_review'))
        q = 'rdf_save(\'{}\', [graph(\'LoggingGraph\')])'.format(path)
        self.prolog_query(q)

    def start_scanning_action(self):
        action_type = 'http://knowrob.org/kb/knowrob.owl#LookingForSomething'
        self.action_graph = ActionGraph(self, action_type, msg='start scanning')

    def start_barcodes_and_separators(self):
        action_type = 'http://knowrob.org/kb/knowrob.owl#LookingForSomething'
        self.action_graph = ActionGraph(self, action_type, msg='detecting barcodes and separators')

    def start_movement(self, action_type='http://knowrob.org/kb/knowrob_common.owl#ArmMovement'):
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(action_type, msg='muh')

    def finish_action(self):
        if self.action_graph is not None:
            self.action_graph = self.action_graph.finish()
