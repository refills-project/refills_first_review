import json
import traceback
from collections import OrderedDict, defaultdict
from multiprocessing import Lock
from rospkg import RosPack

import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np

from tf2_kdl import transform_to_kdl
from visualization_msgs.msg import Marker

from json_prolog import json_prolog
from json_prolog.json_prolog import PrologException
from refills_first_review.tfwrapper import lookup_transform, transform_pose
from refills_first_review.utils import posestamped_to_kdl

MAP = 'map'
SHOP = 'shop'
SHELF_FLOOR = '{}:\'ShelfLayer\''.format(SHOP)
DM_MARKET = 'dmshop'
SHELF_SYSTEM = '{}:\'DMShelfSystem\''.format(DM_MARKET)
SHELF_METERT5 = '{}:\'DMShelfFrameFrontStore\''.format(DM_MARKET)
SHELF_METERT6 = '{}:\'DMShelfFrameH200W100T6\''.format(DM_MARKET)
SHELF_METERT7 = '{}:\'DMShelfFrameH200W100T7\''.format(DM_MARKET)
SHELF_FLOOR_STANDINGT4 = '{}:\'DMShelfLayer4TilesFront\''.format(DM_MARKET)
SHELF_FLOOR_STANDINGT5 = '{}:\'DMShelfLayer5TilesFront\''.format(DM_MARKET)
SHELF_FLOOR_STANDINGT6 = '{}:\'DMShelfLayer6TilesFront\''.format(DM_MARKET)
SHELF_FLOOR_STANDING_GROUNDT5 = '{}:\'DMShelfLayer5TilesBottom\''.format(DM_MARKET)
SHELF_FLOOR_STANDING_GROUNDT6 = '{}:\'DMShelfLayer6TilesBottom\''.format(DM_MARKET)
SHELF_FLOOR_STANDING_GROUNDT7 = '{}:\'DMShelfLayer7TilesBottom\''.format(DM_MARKET)
SHELF_FLOOR_MOUNTING = '{}:\'DMShelfLayerMountingFront\''.format(DM_MARKET)
SEPARATOR = '{}:\'DMShelfSeparator4Tiles\''.format(DM_MARKET)
MOUNTING_BAR = '{}:\'DMShelfMountingBar\''.format(DM_MARKET)
BARCODE = '{}:\'DMShelfLabel\''.format(DM_MARKET)
PERCEPTION_AFFORDANCE = '{}:\'DMShelfPerceptionAffordance\''.format(DM_MARKET)

OBJECT_ACTED_ON = '\'http://knowrob.org/kb/knowrob.owl#objectActedOn\''
GOAL_LOCATION = '\'http://knowrob.org/kb/knowrob.owl#goalLocation\''
DETECTED_OBJECT = '\'http://knowrob.org/kb/knowrob.owl#detectedObject\''

action_tree = []

class ActionGraph(object):
    Action = 0
    Motion = 1
    Event = 2
    logging = True

    def __init__(self, knowrob, parent_node=None, previous_node=None, id='', type=Action):
        self.knowrob = knowrob
        self.previous_node = previous_node
        self.parent_node = parent_node
        self.last_sub_action = None
        self.id = id
        self.type = type
        self.status_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    @classmethod
    def unix_time_seconds(cls):
        now = rospy.get_rostime()
        return now.secs + now.nsecs / 1000000000.0

    @classmethod
    def start_experiment(cls, knowrob, action_type):
        q = 'cram_start_situation(\'{}\', \'{}\', R), ' \
            'rdf_assert(R,knowrob:performedBy,donbot:iai_donbot_robot1, \'LoggingGraph\'), ' \
            'rdf_assert(R,knowrob:performedInMap,iaishop:\'IAIShop_0\', \'LoggingGraph\')'.format(action_type, ActionGraph.unix_time_seconds())
        id = knowrob.prolog_query(q)[0]['R']
        return cls(knowrob, id=id)

    def finish(self):
        q = 'cram_finish_action({}, {})'.format(self.id, ActionGraph.unix_time_seconds())
        if self.logging:
            self.knowrob.prolog_query(q)
        global action_tree # type: list
        if len(action_tree) > 0:
            action_tree.pop(-1)
        # self.make_status_text()
        return self.parent_node

    def create_thingy(self, action_class, action_type):
        global action_tree
        if self.last_sub_action is not None and self.last_sub_action.type == action_type:
            previous_thing = self.last_sub_action.id
        else:
            previous_thing = '_'
        q = '{}(\'{}\', \'{}\', {}, R)'.format(self.type_to_cram_start(action_type), action_class,
                                                   str(ActionGraph.unix_time_seconds()),
                                                   previous_thing)
        if self.logging:
            result = self.knowrob.prolog_query(q)[0]['R']
            return result
        else:
            return ''

    def add_sub_thingy(self, action_type, sub_type, object_acted_on=None, goal_location=None, detected_objects=None):
        new_id = self.create_thingy(action_type, sub_type)

        q = 'rdf_assert({}, {}, {}, \'LoggingGraph\')'.format(self.id, self.type_to_sub(sub_type), new_id)
        if self.logging:
            self.knowrob.prolog_query(q)

            if object_acted_on is not None:
                q = 'rdf_assert({}, {}, \'{}\', \'LoggingGraph\')'.format(new_id, OBJECT_ACTED_ON, object_acted_on)
                self.knowrob.prolog_query(q)
            if goal_location is not None:
                if '[' in goal_location:
                    translation = eval(goal_location.split(', ')[-2])
                    rotation = eval(goal_location.split(', ')[-1][:-1])
                    q = 'belief_new_pose(({}, {}), Id),' \
                        'rdf_assert({}, {}, Id, \'LoggingGraph\')'.format(translation, rotation, new_id, GOAL_LOCATION)
                else:
                    q = 'rdf_assert({}, {}, \'{}\', \'LoggingGraph\')'.format(new_id, GOAL_LOCATION, goal_location)
                self.knowrob.prolog_query(q)
            if detected_objects is not None:
                for detected_object in detected_objects:
                    q = 'rdf_assert({}, {}, \'{}\', \'LoggingGraph\')'.format(new_id, DETECTED_OBJECT, detected_object)
                    self.knowrob.prolog_query(q)

        action_tree.append([action_type, object_acted_on])
        # self.make_status_text()
        self.last_sub_action = ActionGraph(knowrob=self.knowrob, parent_node=self, previous_node=self.last_sub_action,
                                           id=new_id, type=sub_type)
        return self.last_sub_action

    def add_sub_action(self, action_type, object_acted_on=None, goal_location=None, detected_objects=None):
        return self.add_sub_thingy(action_type, self.Action, object_acted_on, goal_location, detected_objects)

    def add_sub_event(self, event_type, object_acted_on=None, goal_location=None, detected_objects=None):
        return self.add_sub_thingy(event_type, self.Event, object_acted_on, goal_location, detected_objects)

    def add_sub_motion(self, motion_type, object_acted_on=None, goal_location=None, detected_objects=None):
        return self.add_sub_thingy(motion_type, self.Motion, object_acted_on, goal_location, detected_objects)

    def type_to_sub(self, t):
        if t == self.Action:
            return 'knowrob:subAction'
        if t == self.Motion:
            return 'knowrob:subMotion'
        if t == self.Event:
            return 'knowrob:subEvent'

    def type_to_cram_start(self, t):
        if t == self.Action:
            return 'cram_start_action'
        if t == self.Motion:
            return 'cram_start_motion'
        if t == self.Event:
            return 'cram_start_event'

    def __str__(self):
        return self.id.split('3')[-1]

    def make_status_text(self):
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.pose.position.z = 1.5
        m.ns = 'action_graph'
        m.id = 1337
        m.action = Marker.ADD
        m.type = Marker.TEXT_VIEW_FACING
        m.scale.z = 0.1
        m.color.a = 1
        m.color.r = 1
        m.frame_locked = True
        global action_tree
        text = ''
        for i, ts in enumerate(action_tree):
            if i > 0:
                text += '\n' + '--' * i
            t = ''
            for j, item in enumerate(ts):
                if item is None:
                    continue
                if j > 0:
                    t += ': '
                if '#' in item:
                    item = item.split('#')[-1]
                t += item
            text += t
        m.text = text
        if m.text != '':
            self.status_pub.publish(m)

class KnowRob(object):
    prefix = 'knowrob_wrapper'
    def __init__(self):
        super(KnowRob, self).__init__()
        self.action_graph = None
        # self.read_left_right_json()
        self.separators = {}
        self.perceived_frame_id_map = {}
        self.prolog = json_prolog.Prolog()
        self.print_with_prefix('waiting for knowrob')
        self.prolog.wait_for_service()
        self.print_with_prefix('knowrob showed up')
        self.query_lock = Lock()

    def print_with_prefix(self, txt):
        print(txt)

    def prolog_query(self, q):
        """
        :type q: str
        :rtype: dict
        """
        with self.query_lock:
            self.print_with_prefix('sending {}'.format(q))
            while True:
                try:
                    solutions = [x if x != {} else True for x in self.prolog.query(q).solutions()]
                    break
                except PrologException as e:
                    self.print_with_prefix('exception {}'.format(e))
                    # TODO enable retry?
                    retry = False
                    # cmd = raw_input('retry? [y/n]')
                    # retry = cmd == 'y'
                    if not retry:
                        raise PrologException(e.message)
            # if len(solutions) > 1:
            #     rospy.logwarn('{} returned more than one result'.format(q))
            # elif len(solutions) == 0:
            #     rospy.logwarn('{} returned nothing'.format(q))
            self.print_with_prefix('solutions {}'.format(solutions))
            return solutions

    def pose_to_prolog(self, pose_stamped):
        """
        :type pose_stamped: PoseStamped
        :return: PoseStamped in a form the knowrob likes
        :rtype: str
        """
        return '[\'{}\', _, [{},{},{}], [{},{},{},{}]]'.format(pose_stamped.header.frame_id,
                                                               pose_stamped.pose.position.x,
                                                               pose_stamped.pose.position.y,
                                                               pose_stamped.pose.position.z,
                                                               pose_stamped.pose.orientation.x,
                                                               pose_stamped.pose.orientation.y,
                                                               pose_stamped.pose.orientation.z,
                                                               pose_stamped.pose.orientation.w)

    def prolog_to_pose_msg(self, query_result):
        """
        :type query_result: list
        :rtype: PoseStamped
        """
        ros_pose = PoseStamped()
        ros_pose.header.frame_id = query_result[0]
        ros_pose.pose.position = Point(*query_result[2])
        ros_pose.pose.orientation = Quaternion(*query_result[3])
        return ros_pose

    def read_left_right_json(self):
        self.path_to_json = rospy.get_param('~path_to_json')
        self.left_right_dict = {}
        with open(self.path_to_json, 'r') as f:
            self.left_right_dict.update(json.load(f))

    def is_left(self, shelf_system_id):
        return self.left_right_dict[shelf_system_id] == u'left'

    def is_right(self, shelf_system_id):
        return self.left_right_dict[shelf_system_id] == u'right'

    def get_shelf_system_ids(self):
        """
        :return: list of str
        :rtype: list
        """
        ids = []
        ids.extend(self.get_objects(SHELF_METERT6).keys())
        ids.extend(self.get_objects(SHELF_METERT7).keys())
        ids.extend(self.get_objects(SHELF_METERT5).keys())
        return ids

    def is_6tile_system(self, shelf_system_id):
        return shelf_system_id in self.get_objects(SHELF_METERT6)

    def is_7tile_system(self, shelf_system_id):
        return shelf_system_id in self.get_objects(SHELF_METERT7)

    def get_bottom_layer_type(self, shelf_system_id):
        if self.is_6tile_system(shelf_system_id):
            return SHELF_FLOOR_STANDING_GROUNDT6
        if self.is_7tile_system(shelf_system_id):
            return SHELF_FLOOR_STANDING_GROUNDT7
        else:
            return SHELF_FLOOR_STANDING_GROUNDT5

    def get_standing_layer_type(self, shelf_system_id):
        if self.is_6tile_system(shelf_system_id):
            return SHELF_FLOOR_STANDINGT5
        if self.is_7tile_system(shelf_system_id):
            return SHELF_FLOOR_STANDINGT6
        else:
            return SHELF_FLOOR_STANDINGT4

    def get_shelf_layer_from_system(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :return: returns dict mapping floor id to pose ordered from lowest to highest
        :rtype: dict
        """
        q = 'rdf_has(\'{}\', knowrob:properPhysicalParts, Floor), ' \
            'rdfs_individual_of(Floor, {}), ' \
            'object_perception_affordance_frame_name(Floor, Frame).'.format(shelf_system_id, SHELF_FLOOR)

        solutions = self.prolog_query(q)
        floors = []
        shelf_frame_id = self.get_perceived_frame_id(shelf_system_id)
        for solution in solutions:
            floor_id = solution['Floor'].replace('\'', '')
            floor_pose = lookup_transform(shelf_frame_id, solution['Frame'].replace('\'', ''))
            floors.append((floor_id, floor_pose))
        floors = list(sorted(floors, key=lambda x: -x[1].pose.position.z))
        self.floors = OrderedDict(floors)
        return self.floors

    def get_facing_ids_from_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return:
        :rtype: OrderedDict
        """
        shelf_system_id = self.get_shelf_system_from_layer(shelf_layer_id)
        q = 'findall([F, P], (shelf_facing(\'{}\', F),belief_at(F, P)), Fs).'.format(shelf_layer_id)
        solutions = self.prolog_query(q)[0]
        facings = []
        for facing_id, pose in solutions['Fs']:
            facing_pose = self.prolog_to_pose_msg(pose)
            facing_pose = transform_pose(self.get_perceived_frame_id(shelf_layer_id), facing_pose)
            facings.append((facing_id, facing_pose))
        is_left = 1 if self.is_left(shelf_system_id) else -1
        facings = list(sorted(facings, key=lambda x: x[1].pose.position.x * is_left))
        return OrderedDict(facings)

    def shelf_system_exists(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: bool
        """
        return shelf_system_id in self.get_shelf_system_ids()

    def shelf_layer_exists(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: bool
        """
        q = 'shelf_layer_frame(\'{}\', _).'.format(shelf_layer_id)
        return len(self.prolog_query(q)) != 0

    def facing_exists(self, facing_id):
        """
        :type facing_id: str
        :rtype: bool
        """
        q = 'shelf_facing(L, \'{}\').'.format(facing_id)
        return len(self.prolog_query(q)) != 0

    def get_objects(self, object_type):
        """
        Ask knowrob for a specific type of objects
        :type object_type: str
        :return: all objects of the given type
        :rtype: dict
        """
        objects = OrderedDict()
        q = 'rdfs_individual_of(R, {}).'.format(object_type)
        solutions = self.prolog_query(q)
        for solution in solutions:
            object_id = solution['R'].replace('\'', '')
            pose_q = 'belief_at(\'{}\', R).'.format(object_id)
            believed_pose = self.prolog_query(pose_q)[0]['R']
            ros_pose = PoseStamped()
            ros_pose.header.frame_id = believed_pose[0]
            ros_pose.pose.position = Point(*believed_pose[2])
            ros_pose.pose.orientation = Quaternion(*believed_pose[3])
            objects[str(object_id)] = ros_pose
        return objects

    def get_perceived_frame_id(self, object_id):
        """
        :type object_id: str
        :return: the frame_id of an object according to the specifications in our wiki.
        :rtype: str
        """
        if object_id not in self.perceived_frame_id_map:
            q = 'object_perception_affordance_frame_name(\'{}\', F)'.format(object_id)
            self.perceived_frame_id_map[object_id] = self.prolog_query(q)[0]['F'].replace('\'', '')
        return self.perceived_frame_id_map[object_id]

    def get_object_frame_id(self, object_id):
        """
        :type object_id: str
        :return: frame_id of the center of mesh.
        :rtype: str
        """
        q = 'object_frame_name(\'{}\', R).'.format(object_id)
        return self.prolog_query(q)[0]['R'].replace('\'', '')

    # floor
    def add_shelf_layers(self, shelf_system_id, floor_heights):
        """
        :param shelf_system_id: layers will be attached to this shelf system.
        :type shelf_system_id: str
        :param floor_heights: heights of the detects layers, list of floats
        :type floor_heights: list
        :return: TODO
        :rtype: bool
        """
        # TODO return false on failure
        # TODO support different types of shelf layer
        for i, height in enumerate(sorted(floor_heights)):
            if i == 0:
                layer_type = self.get_bottom_layer_type(shelf_system_id)
            else:
                layer_type = self.get_standing_layer_type(shelf_system_id)
            q = 'belief_shelf_part_at(\'{}\', {}, {}, R)'.format(shelf_system_id, layer_type, height[-1])
            self.prolog_query(q)
        return True

    def update_shelf_layer_position(self, shelf_layer_id, separators):
        """
        :type shelf_layer_id: str
        :type separators: list of PoseStamped, positions of separators
        """
        new_floor_height = np.mean([transform_pose(self.get_perceived_frame_id(shelf_layer_id), p).pose.position.z for p in separators])
        current_floor_pose = lookup_transform(MAP, self.get_object_frame_id(shelf_layer_id))
        current_floor_pose.pose.position.z += new_floor_height  # - 0.015
        q = 'belief_at_update(\'{}\', {})'.format(shelf_layer_id, self.pose_to_prolog(current_floor_pose))
        self.prolog_query(q)

    def add_separators(self, shelf_layer_id, separators):
        """
        :param shelf_layer_id: separators will be attached to this shelf layer.
        :type shelf_layer_id: str
        :param separators: list of PoseStamped, positions of separators
        :return:
        """
        # TODO check success
        for p in separators:
            q = 'belief_shelf_part_at(\'{}\', {}, {}, _)'.format(shelf_layer_id, SEPARATOR, p.pose.position.x)
            try:
                self.prolog_query(q)
            except Exception as e:
                traceback.print_exc()
                return False
        return True

    def add_barcodes(self, shelf_layer_id, barcodes):
        """
        :param shelf_layer_id: barcodes will be attached to this shelf layer
        :type shelf_layer_id: str
        :param barcodes: dict mapping barcode to PoseStamped. make sure it relative to shelf layer, everything but x ignored
        :type barcodes: dict
        """
        # TODO check success
        for barcode, p in barcodes.items():
            q = 'belief_shelf_barcode_at(\'{}\', {}, dan(\'{}\'), {}, _)'.format(shelf_layer_id, BARCODE,
                                                                                 barcode, p.pose.position.x)
            self.prolog_query(q)

    def get_all_product_dan(self):
        """
        :return: list of str
        :rtype: list
        """
        q = 'findall(DAN,rdf_has_prolog(AN,shop:dan,DAN), DANS)'
        dans = self.prolog_query(q)[0]['DANS']
        return dans

    def add_objects(self, facing_id, number):
        """
        Adds objects to the facing whose type is according to the barcode.
        :type facing_id: str
        :type number: int
        """
        for i in range(number):
            q = 'product_spawn_front_to_back(\'{}\', ObjId)'.format(facing_id)
            self.prolog_query(q)

    def save_beliefstate(self, path=None):
        """
        :type path: str
        """
        if path is None:
            path = '{}/data/beliefstate.owl'.format(RosPack().get_path('refills_first_review'))
        q = 'rdf_save(\'{}\', belief_state)'.format(path)
        self.prolog_query(q)

    def get_shelf_layer_width(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', D, W, H).'.format(shelf_layer_id)
        solution = self.prolog_query(q)[0]
        width = solution['W']
        return width

    def get_shelf_system_width(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', D, W, H).'.format(shelf_system_id)
        solution = self.prolog_query(q)[0]
        width = solution['W']
        return width

    def get_shelf_system_height(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', D, W, H).'.format(shelf_system_id)
        solution = self.prolog_query(q)[0]
        height = solution['H']
        return height

    def get_shelf_system_from_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: str
        """
        q = 'shelf_layer_frame(\'{}\', Frame).'.format(shelf_layer_id)
        return self.prolog_query(q)[0]['Frame'][1:-1]

    def get_shelf_layer_from_facing(self, facing_id):
        """
        :type facing_id: str
        :rtype: str
        """
        q = 'shelf_facing(Layer, \'{}\').'.format(facing_id)
        return self.prolog_query(q)[0]['Layer'][1:-1]

    def get_shelf_layer_above(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return: shelf layer above or None if it does not exist.
        :rtype: str
        """
        q = 'shelf_layer_above(\'{}\', Above).'.format(shelf_layer_id)
        solutions = self.prolog_query(q)
        if len(solutions) > 0:
            return solutions[0]['Above'][1:-1]

    def reset_beliefstate(self):
        """
        :rtype: bool
        """
        q = 'belief_forget, retractall(owl_parser:owl_file_loaded(Path))'
        return self.prolog_query(q)[0]

    def load_owl(self, path):
        """
        :type path: str
        :rtype: bool
        """
        q = 'belief_parse(\'{}\')'.format(path)
        return self.prolog_query(q)[0]

    def remove_http_shit(self, s):
        return s.split('#')[-1].split('\'')[0]

    def load_barcode_to_mesh_map(self):
        self.barcode_to_mesh = json.load(open('../../data/barcode_to_mesh.json'))


    def add_shelf_system(self):
        q = 'belief_new_object({}, R), rdf_assert(R, knowrob:describedInMap, iaishop:\'IAIShop_0\', belief_state)'.format(
            SHELF_SYSTEM)
        shelf_system_id = self.prolog_query(q)[0]['R'].replace('\'', '')
        return shelf_system_id

    # shelves
    def add_shelves(self, shelf_system_id, shelves):
        # TODO failure handling
        for name, pose in shelves.items(): #type: (str, PoseStamped)
            q = 'belief_new_object({}, ID), ' \
                'rdf_assert(\'{}\', knowrob:properPhysicalParts, ID, belief_state),' \
                'object_affordance_static_transform(ID, A, [_,_,T,R]),' \
                'rdfs_individual_of(A, {})'.format(SHELF_METERT5, shelf_system_id, PERCEPTION_AFFORDANCE)
            solutions = self.prolog_query(q)[0]

            shelf_transform = posestamped_to_kdl(pose)
            shelf_offset = PyKDL.Frame(PyKDL.Vector(-solutions['T'][0], -solutions['T'][1], -solutions['T'][2]))
            offset_shelf_transform = shelf_transform * shelf_offset # type: PyKDL.Frame
            new_pose = PoseStamped()
            new_pose.header = pose.header
            new_pose.pose.position = Point(*offset_shelf_transform.p)
            new_pose.pose.orientation = Quaternion(*offset_shelf_transform.M.GetQuaternion())

            object_id = solutions['ID'].replace('\'', '')
            q = 'belief_at_update(\'{}\', {})'.format(object_id, self.pose_to_prolog(new_pose))
            solutions = self.prolog_query(q)

        return True

    # def get_shelves(self):
    #     return self.get_objects(SHELF_METER)

    # def get_perceived_frame_id(self, object_id):
    #     if object_id not in self.perceived_frame_id_map:
    #         q = 'object_perception_affordance_frame_name(\'{}\', F)'.format(object_id)
    #         self.perceived_frame_id_map[object_id] = self.prolog_query(q)[0]['F'].replace('\'', '')
    #     return self.perceived_frame_id_map[object_id]
    #
    # def get_object_frame_id(self, object_id):
    #     q = 'object_frame_name(\'{}\', R).'.format(object_id)
    #     return self.prolog_query(q)[0]['R'].replace('\'', '')

    # floor
    # def add_shelf_floors(self, shelf_id, floors):
    #     for position in floors:
    #         if position[1] < 0.13:
    #             if position[2] < 0.2:
    #                 layer_type = SHELF_FLOOR_STANDING_GROUND
    #             else:
    #                 layer_type = SHELF_FLOOR_STANDING
    #         else:
    #             layer_type = SHELF_FLOOR_MOUNTING
    #         q = 'belief_shelf_part_at(\'{}\', {}, {}, R)'.format(shelf_id, layer_type, position[-1])
    #         self.prolog_query(q)
    #     return True

    def get_floor_ids(self, shelf_id):
        q = 'rdf_has(\'{}\', knowrob:properPhysicalParts, Floor), ' \
            'rdfs_individual_of(Floor, {}), ' \
            'object_perception_affordance_frame_name(Floor, Frame).'.format(shelf_id, SHELF_FLOOR)

        solutions = self.prolog_query(q)
        floors = []
        shelf_frame_id = self.get_perceived_frame_id(shelf_id)
        for solution in solutions:
            floor_id = solution['Floor'].replace('\'', '')
            floor_pose = lookup_transform(shelf_frame_id, solution['Frame'].replace('\'', ''))
            if floor_pose.pose.position.z < 1.3:
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
        return self.get_floor_position(floor_id).pose.position.z < 0.2

    def is_hanging_foor(self, floor_id):
        q = 'rdfs_individual_of(\'{}\', {})'.format(floor_id, SHELF_FLOOR_MOUNTING)
        solutions = self.prolog_query(q)
        return len(solutions) > 0

    def is_normal_floor(self, floor_id):
        return not self.is_bottom_floor(floor_id) and not self.is_hanging_foor(floor_id)

    # def add_separators(self, floor_id, separators):
    #     for p in separators:
    #         q = 'belief_shelf_part_at(\'{}\', {}, {}, _)'.format(floor_id, SEPARATOR, p.pose.position.x)
    #         self.prolog_query(q)
    #     return True
    #
    # def add_barcodes(self, floor_id, barcodes):
    #     for barcode, p in barcodes.items():
    #         q = 'belief_shelf_barcode_at(\'{}\', {}, dan(\'{}\'), {}, _)'.format(floor_id, BARCODE,
    #                                                                              barcode, p.pose.position.x)
    #         self.prolog_query(q)

    def add_separators_and_barcodes(self, floor_id, separators, barcodes):
        # update floor height
        new_floor_height = np.mean([transform_pose(
            self.get_perceived_frame_id(floor_id), p).pose.position.z for p in separators])
        current_floor_pose = lookup_transform(MAP, self.get_object_frame_id(floor_id))
        current_floor_pose.pose.position.z += new_floor_height #- 0.015
        q = 'belief_at_update(\'{}\', {})'.format(floor_id, self.pose_to_prolog(current_floor_pose))
        # self.prolog_query(q)

        for p in separators:
            q = 'belief_shelf_part_at(\'{}\', {}, norm({}), _)'.format(floor_id, SEPARATOR, p.pose.position.x)
            self.prolog_query(q)

        for barcode, p in barcodes.items():
            q = 'belief_shelf_barcode_at(\'{}\', {}, dan(\'{}\'), norm({}), _)'.format(floor_id, BARCODE, barcode,
                                                                                       p.pose.position.x)
            self.prolog_query(q)

        self.start_shelf_separator_perception(self.get_separators(floor_id))
        self.finish_action()
        self.start_shelf_label_perception(self.get_barcodes(floor_id))
        self.finish_action()

    def get_separators(self, floor_id):
        q = 'findall(S, shelf_layer_separator(\'{}\', S), Ss)'.format(floor_id)
        solutions = self.prolog_query(q)
        return solutions[0]['Ss']

    def get_mounting_bars(self, floor_id):
        q = 'findall(S, shelf_layer_mounting_bar(\'{}\', S), Ss)'.format(floor_id)
        solutions = self.prolog_query(q)
        return solutions[0]['Ss']

    def get_barcodes(self, floor_id):
        q = 'findall(S, shelf_layer_label(\'{}\', S), Ss)'.format(floor_id)
        solutions = self.prolog_query(q)
        return solutions[0]['Ss']

    def add_mounting_bars_and_barcodes(self, floor_id, separators, barcodes):
        # if len(separators) > 0:
        #     for p in separators:
        #         q = 'belief_shelf_part_at(\'{}\', {}, norm({}), _)'.format(floor_id, MOUNTING_BAR, p.pose.position.x)
        #         self.prolog_query(q)
        # else:
        for barcode, p in barcodes.items():
            q = 'belief_shelf_part_at(\'{}\', {}, norm({}), _)'.format(floor_id, MOUNTING_BAR, p.pose.position.x+0.01)
            self.prolog_query(q)

        for barcode, p in barcodes.items():
            q = 'belief_shelf_barcode_at(\'{}\', {}, dan(\'{}\'), norm({}), _)'.format(floor_id, BARCODE, barcode,
                                                                                       p.pose.position.x)
            self.prolog_query(q)

        self.start_shelf_bar_perception(self.get_mounting_bars(floor_id))
        self.finish_action()
        self.start_shelf_label_perception(self.get_barcodes(floor_id))
        self.finish_action()

    def get_facings(self, floor_id):
        q = 'findall([F, P, W, L], (shelf_facing(\'{}\', F), ' \
            'shelf_facing_product_type(F,P), ' \
            'comp_facingWidth(F,literal(type(_, W))), ' \
            '(rdf_has(F, shop:leftSeparator, L); rdf_has(F, shop:mountingBarOfFacing, L))),' \
            'Facings).'.format(floor_id)
        solutions = self.prolog_query(q)[0]
        facings = {}
        for facing_id, product, width, left_separator_id in solutions['Facings']:
            facing_pose = lookup_transform(self.get_perceived_frame_id(floor_id),
                                                   self.get_object_frame_id(facing_id))
            facings[facing_id] = (facing_pose, product, float(width), left_separator_id)
        return facings

    def add_object(self, facing_id):
        q = 'product_spawn_front_to_back(\'{}\', ObjId)'.format(facing_id)
        self.prolog_query(q)

    # def save_beliefstate(self, path=None):
    #     if path is None:
    #         path = '{}/data/beliefstate.owl'.format(RosPack().get_path('refills_first_review'))
    #     q = 'rdf_save(\'{}\', belief_state)'.format(path)
    #     self.prolog_query(q)

#-----------------------------------------------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------

    def save_action_graph(self, path=None):
        if path is None:
            path = '{}/data/actions.owl'.format(RosPack().get_path('refills_first_review'))
        q = 'rdf_save(\'{}\', [graph(\'LoggingGraph\')])'.format(path)
        self.prolog_query(q)

    def start_everything(self):
        a = 'http://knowrob.org/kb/knowrob.owl#RobotExperiment'
        self.action_graph = ActionGraph.start_experiment(self, a)

    def start_shelf_system_mapping(self, object_acted_on=None):
        """
        :param object_acted_on: shelf system id
        :return:
        """
        a = 'http://knowrob.org/kb/shop.owl#ShelfSystemMapping'
        self.action_graph = self.action_graph.add_sub_action(self, a, object_acted_on=object_acted_on)

    def start_shelf_frame_mapping(self, object_acted_on=None):
        a = 'http://knowrob.org/kb/shop.owl#ShelfFrameMapping'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a, object_acted_on=object_acted_on)

    def start_shelf_layer_mapping(self, object_acted_on=None):
        """
        :param object_acted_on: floor_id
        :return:
        """
        a = 'http://knowrob.org/kb/shop.owl#ShelfLayerMapping'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a, object_acted_on=object_acted_on)

    def start_finding_shelf_layer(self):
        a = 'http://knowrob.org/kb/shop.owl#FindingShelfLayer'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a)

    def start_finding_shelf_layer_parts(self):
        a = 'http://knowrob.org/kb/shop.owl#FindingShelfLayerParts'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a)

    def start_shelf_layer_perception(self, detected_objects=None):
        """
        :param detected_objects: list of floor_id
        :return:
        """
        a = 'http://knowrob.org/kb/shop.owl#ShelfLayerPerception'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_event(a, detected_objects=detected_objects)

    def start_shelf_layer_counting(self):
        a = 'http://knowrob.org/kb/shop.owl#ShelfLayerCounting'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a)

    def start_move_to_shelf_frame(self, goal_location=None):
        """
        :param goal_location: shelf id
        :return:
        """
        a = 'http://knowrob.org/kb/shop.owl#MoveToShelfFrame'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a, goal_location=goal_location)

    def start_move_to_shelf_frame_end(self, goal_location=None):
        """
        :param goal_location: pose
        :return:
        """
        a = 'http://knowrob.org/kb/shop.owl#MoveToShelfFrameEnd'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a, goal_location=goal_location)

    def start_move_to_shelf_layer(self, goal_location=None):
        """
        :param goal_location: floor id
        :return:
        """
        a = 'http://knowrob.org/kb/shop.owl#MoveToShelfLayer'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a, goal_location=goal_location)

    def start_looking_at_location(self, goal_location=None):
        """
        :param goal_location: pose
        :return:
        """
        a = 'http://knowrob.org/kb/knowrob.owl#LookingAtLocation'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_action(a, goal_location=goal_location)

    def start_base_movement(self, goal_location=None):
        """
        :param goal_location: pose
        :return:
        """
        a = 'http://knowrob.org/kb/motions.owl#BaseMovement'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_motion(a, goal_location=goal_location)

    def start_head_movement(self, goal_location=None):
        """
        :param goal_location: pose
        :return:
        """
        a = 'http://knowrob.org/kb/motions.owl#HeadMovement'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_motion(a, goal_location=goal_location)

    def start_shelf_separator_perception(self, detected_objects=None):
        a = 'http://knowrob.org/kb/shop.owl#ShelfSeparatorPerception'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_event(a, detected_objects=detected_objects)

    def start_shelf_bar_perception(self, detected_objects=None):
        a = 'http://knowrob.org/kb/shop.owl#ShelfBarPerception'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_event(a, detected_objects=detected_objects)

    def start_shelf_label_perception(self, detected_objects=None):
        a = 'http://knowrob.org/kb/shop.owl#ShelfLabelPerception'
        if self.action_graph is not None:
            self.action_graph = self.action_graph.add_sub_event(a, detected_objects=detected_objects)

    def finish_action(self):
        if self.action_graph is not None:
            self.action_graph = self.action_graph.finish()
