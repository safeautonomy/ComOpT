from collections import defaultdict, namedtuple
from comopt.proto_lib.modules.map.proto.map_overlap_pb2 import CrosswalkOverlapInfo, Overlap
from comopt.proto_lib.modules.map.proto.map_lane_pb2 import Lane
from comopt.proto_lib.modules.map.proto.map_crosswalk_pb2 import Crosswalk
import sys, os, math, logging, json
from typing import Dict, Iterable, Tuple, Union

from comopt.geometry import Vector, Segment, SLPoint, check_intersect, lerp, clamp, intersect_point
from comopt.utils import DictObj, Match, check_type

from comopt.map import MapParser, map_config
from .kdtree import KDTree, SegmentInfo
from .visualizer import MapVisualizerServer
import comopt.simulator as simulator

from functools import cached_property

from enum import Enum

from matplotlib import pyplot as plt

class LaneType(Enum):
    NONE = 1
    CITY_DRIVING = 2
    BIKING = 3
    SIDEWALK = 4
    PARKING = 5
    SHOULDER = 6
    

class LaneTurn(Enum):
    NO_TURN  = 1
    LEFT_TURN = 2
    RIGHT_TURN = 3
    U_TURN = 4

class LaneBoundType(Enum):
    DOTTED_WHITE = 1
    DOTTED_YELLOW = 2
    DOUBLE_YELLOW = 3
    SOLID_WHITE = 4
    SOLID_YELLOW = 5
    CURB = 6
    UNKNOWN = 7


class OutOfRoadError(Exception):
    pass

class Crosswalk:
    def __init__(self, map, crosswalk_proto):
        self._id = crosswalk_proto.id.id
        self.crosswalk_proto = crosswalk_proto
        self.map = map
        self._covered_lanes = set()
        
        
    @property
    def id(self):
        return self._id

    @cached_property
    def coverd_lanes(self):
        for obj in self.map.overlaps[self]:
            if isinstance(obj, Lane):
                self._covered_lanes.add(obj)
        return self._covered_lanes

    def __eq__(self, crosswalk):
        return self.id == crosswalk.id

    def __hash__(self) -> int:
        return hash(self.id)

    def cover_range(self, lane):
        if isinstance(lane, str):
            lane = self.map.lanes[lane]
        assert isinstance(lane, Lane)
        for obj, overlap in self.map.overlaps[lane].items():
            if obj == self:
                return overlap.cover_range
        return None

    def __str__(self):
        return f'Crosswalk<{self.id}>'

    def __repr__(self):
        return self.__str__()
    
    @cached_property
    def sides(self):
        assert len(self.crosswalk_proto.polygon.point) == 4
        polygon_proto = self.crosswalk_proto.polygon
        points = [Vector(point.x, point.y) for point in (list(polygon_proto.point) + [polygon_proto.point[0]])]

        polygon = [Segment(points[i], points[(i+1)%4]) for i in range(4)]

        sides = {}
        for i in range(4):
            crosswalk_side = polygon[i]
            lanes_info = [
                self.map.get_closest_lane(crosswalk_side.p1, False),
                self.map.get_closest_lane(crosswalk_side.p2, False),
            ]

            lanes_info = [item for item in lanes_info if not item is None]
            if len(lanes_info) > 0:
                intersect = False
                for lane_info in lanes_info:
                    for lane_seg in lane_info.obj.central_segments:
                        if check_intersect(crosswalk_side, lane_seg):
                            intersect = True
                            break
                    if intersect: break

                if not intersect:
                    sides[i] = (crosswalk_side, 'left' if list(lanes_info)[0].l > 0 else 'right')

        assert 1 < len(sides) < 4

        if len(sides) == 3:
            no_side_index = 10 - sum(sides.keys())
            sides.pop((no_side_index + 2) % 4)
        CrosswalkSide = namedtuple('CrosswalkSide', ['left', 'right'])
        for side_info in sides.values():
            if side_info[1] == 'left':
                left_side = side_info[0]
            elif side_info[1] == 'right':
                right_side = side_info[0]
        return CrosswalkSide(left = left_side, right = right_side)

class TrafficLight:
    def __init__(self, map, traffic_light_proto):
        self.map = map
        self._id = traffic_light_proto.id.id
        self._position = sum([Vector(point.x, point.y) for point in traffic_light_proto.boundary.point], start=Vector()) / 4
        self._height = sum([point.z for point in traffic_light_proto.boundary.point])
        self._proto = traffic_light_proto
        
        points = self._proto.stop_line[0].segment[0].line_segment.point
        num = len(points)
        self._stop_segments = tuple(Segment(Vector(points[i].x, points[i].y), Vector(points[i+1].x, points[i+1].y)) for i in range(num-1))

    @property
    def id(self):
        return self._id

    @property
    def position(self):
        return self._position

    @property
    def stop_segments(self):
        return self._stop_segments

    @property
    def height(self):
        return self._height

    @cached_property
    def control_lanes(self):
        ret = []
        for obj in self.map.overlaps[self]:
            if isinstance(obj, Lane):
                ret.append(obj)
        return tuple(ret)

    def dis_to_ground(self, position:Vector):
        return math.sqrt((position.x - self.position.x) ** 2 + (position.y - self.position.y) ** 2 + self.height ** 2)

    def __hash__(self) -> int:
        return hash(self.id)
    
    def __str__(self):
        return f'TrafficLight<{self.id}>'

    def __repr__(self):
        return self.__str__()

class LaneSpawn:
    def __init__(self, lane, s):
        self.lane = lane
        self.s = s
    
    def get_state(self, velocity = 0):
        return self.lane.get_normal_state_by_s(self.s, velocity)

    def __eq__(self, lane_spawn):
        return self.lane == lane_spawn.lane and self.s == lane_spawn.s

    def __hash__(self) -> int:
        return hash((self.lane, self.s))

    def __str__(self):
        return f'LaneSpawn<{self.lane.id}, {self.s}>'

    def __repr__(self):
        return self.__str__()

class Lane:
        @check_type
        def __init__(self, map, lane_proto):
            self.map = map
            self.lane_proto = lane_proto
            self._id = lane_proto.id.id
            self.central_points = tuple(Vector(point.x, point.y) for point in lane_proto.central_curve.segment[0].line_segment.point)
            self.central_segments = tuple(Segment(self.central_points[i], self.central_points[i+1]) for i in range(len(self.central_points)-1))
        
        @property
        def id(self):
            return self._id

        @property
        def predecessors(self):
            return tuple(self.map.get_lane_by_id(id.id) for id in self.lane_proto.predecessor_id)

        @property
        def successors(self):
            return tuple(self.map.get_lane_by_id(id.id) for id in self.lane_proto.successor_id)

        @property
        def left_forward_neighbors(self):
            return tuple(self.map.get_lane_by_id(id.id) for id in self.lane_proto.left_neighbor_forward_lane_id)

        @property
        def right_forward_neighbors(self):
            return tuple(self.map.get_lane_by_id(id.id) for id in self.lane_proto.right_neighbor_forward_lane_id)

        @property
        def left_reverse_neighbors(self):
            return tuple(self.map.get_lane_by_id(id.id) for id in self.lane_proto.left_neighbor_reverse_lane_id)

        @property
        def right_reverse_neighbors(self):
            return tuple(self.map.get_lane_by_id(id.id) for id in self.lane_proto.right_neighbor_reverse_lane_id)

        @cached_property
        def neighbors(self):
            neighbors = set()
            neighbors |= set(self.predecessors)
            neighbors |= set(self.successors)
            neighbors |= set(self.left_forward_neighbors)
            neighbors |= set(self.right_forward_neighbors)
            bound_type = self.bound_type
            if bound_type.left != LaneBoundType.DOUBLE_YELLOW and bound_type.left != LaneBoundType.SOLID_YELLOW:
                neighbors |= set(self.left_reverse_neighbors)
            if bound_type.right != LaneBoundType.DOUBLE_YELLOW and bound_type.right != LaneBoundType.SOLID_YELLOW:
                neighbors |= set(self.right_reverse_neighbors)
            return neighbors
            

        @property
        def speed_limit(self):
            return self.lane_proto.speed_limit

        @property
        def type(self):
            if self.lane_proto.type == self.lane_proto.LaneType.NONE:
                return LaneType.NONE
            elif self.lane_proto.type == self.lane_proto.LaneType.CITY_DRIVING:
                return LaneType.CITY_DRIVING
            elif self.lane_proto.type == self.lane_proto.LaneType.BIKING:
                return LaneType.BIKING
            elif self.lane_proto.type == self.lane_proto.LaneType.SIDEWALK:
                return LaneType.SIDEWALK
            elif self.lane_proto.type == self.lane_proto.LaneType.PARKING:
                return LaneType.PARKING
            elif self.lane_proto.type == self.lane_proto.LaneType.SHOULDER:
                return LaneType.SHOULDER
        
        @property
        def turn(self):
            if self.lane_proto.turn  == self.lane_proto.LaneTurn.U_TURN:
                return LaneTurn.U_TURN
            elif self.lane_proto.turn == self.lane_proto.LaneTurn.LEFT_TURN:
                return LaneTurn.LEFT_TURN
            elif self.lane_proto.turn == self.lane_proto.LaneTurn.RIGHT_TURN:
                return LaneTurn.RIGHT_TURN
            else:
                return LaneTurn.NO_TURN

        @property
        def max_deflection(self):
            max_ret = 0
            for seg1 in self.central_segments:
                for seg2 in self.central_segments:
                    ret = seg1.vec.angle_with(seg2.vec)
                    if ret > max_ret:
                        max_ret = ret
            return max_ret

        @cached_property
        def crosswalks(self) -> Tuple['CrosswalkLaneOverlap']:
            ret = []
            for obj, overlap in self.map.overlaps[self].items():
                if isinstance(obj, Crosswalk):
                    ret.append(overlap)
            return tuple(ret)

        @cached_property
        def traffic_lights(self) -> Tuple[TrafficLight]:
            ret = []
            for obj, overlap in self.map.overlaps[self].items():
                if isinstance(obj, TrafficLight):
                    ret.append(overlap.traffic_light)
            return tuple(ret)

        def get_spawns(self, step, start_offset=5, end_offset=5):
            spwan_s = start_offset
            ret = []
            while spwan_s < self.length - end_offset:
                ret.append(LaneSpawn(self, spwan_s))
                spwan_s += step
            return tuple(ret)

        def get_segment_by_s(self, s:Union[int, float]):
            for segment in self.central_segments:
                if segment.length >= s:
                    return segment
                else:
                    s -= segment.length
            return None

        def get_normal_state_by_s(self, s, velocity=0):
            if abs(s) > self.length:
                return None
            if s < 0:
                s = self.length + s
            for segment in self.central_segments:
                if segment.length >= s:
                    position = segment.lerp(s / segment.length)
                    heading = segment.vec.angle
                    return simulator.AgentState(position=position, heading=heading, velocity=segment.vec.unit * velocity)
                else:
                    s -= segment.length
            return None

        def intersect_points_with_lane(self, lane):
            ret = []
            for seg1 in self.central_segments:
                for seg2 in lane.central_segments:
                    point = intersect_point(seg1, seg2)
                    if not point is None:
                        ret.append(point)
            return tuple(ret)

        # @check_type
        def get_xy_by_sl(self, slpoint:SLPoint):
            remain = slpoint.s
            
            point = None
            for segment in self.central_segments:
                if segment.length > remain:
                    point = segment.lerp(remain / segment.length)
                    break
                remain -= segment.length
            else:
                raise Exception('s out of lane')
            
            return point + point.left_unit * slpoint.l

        @property
        def bound(self):
            bound = dict()
            BoundInfo = namedtuple('BoundInfo', ['min_l', 'max_l'])
            for left_sample, right_sample in zip(self.lane_proto.left_sample, self.lane_proto.right_sample):
                bound[left_sample.s] = BoundInfo(min_l = -right_sample.width-0.5, max_l=left_sample.width+0.5)
            return bound

        @property
        def bound_type(self):
            BoundType = namedtuple('BoundTypeInfo', ['left', 'right'])
            ProtoBoundType = self.lane_proto.left_boundary.boundary_type[0].Type
            type = self.lane_proto.left_boundary.boundary_type[0].types[0]
            left_type = LaneBoundType.UNKNOWN
            if type == ProtoBoundType.UNKNOWN:
                left_type = LaneBoundType.UNKNOWN
            elif type == ProtoBoundType.DOTTED_YELLOW:
                left_type = LaneBoundType.DOTTED_YELLOW
            elif type == ProtoBoundType.DOTTED_WHITE:
                left_type = LaneBoundType.DOTTED_WHITE
            elif type == ProtoBoundType.SOLID_YELLOW:
                left_type = LaneBoundType.SOLID_YELLOW
            elif type == ProtoBoundType.SOLID_WHITE:
                left_type = LaneBoundType.SOLID_WHITE
            elif type == ProtoBoundType.DOUBLE_YELLOW:
                left_type = LaneBoundType.DOUBLE_YELLOW
            elif type == ProtoBoundType.CURB:
                left_type = LaneBoundType.CURB

            type = self.lane_proto.right_boundary.boundary_type[0].types[0]
            right_type = LaneBoundType.UNKNOWN
            if type == ProtoBoundType.UNKNOWN:
                right_type = LaneBoundType.UNKNOWN
            elif type == ProtoBoundType.DOTTED_YELLOW:
                right_type = LaneBoundType.DOTTED_YELLOW
            elif type == ProtoBoundType.DOTTED_WHITE:
                right_type = LaneBoundType.DOTTED_WHITE
            elif type == ProtoBoundType.SOLID_YELLOW:
                right_type = LaneBoundType.SOLID_YELLOW
            elif type == ProtoBoundType.SOLID_WHITE:
                right_type = LaneBoundType.SOLID_WHITE
            elif type == ProtoBoundType.DOUBLE_YELLOW:
                right_type = LaneBoundType.DOUBLE_YELLOW
            elif type == ProtoBoundType.CURB:
                right_type = LaneBoundType.CURB
            
            return BoundType(left=left_type, right=right_type)

        def bound_types(self, s):
            BoundTypeInfo = namedtuple('BoundTypeInfo', ['left', 'right'])
            side = 'l'
            left_type = None
            right_type = None
            for boundary in [self.lane_proto.left_boundary, self.lane_proto.right_boundary]:
                for bound_type in boundary.boundary_type:
                    if bound_type.s <= s:
                        types = []
                        for type in bound_type.types:
                            if type == bound_type.Type.UNKNOWN:
                                types.append(LaneBoundType.UNKNOWN)
                            elif type == bound_type.Type.DOTTED_YELLOW:
                                types.append(LaneBoundType.DOTTED_YELLOW)
                            elif type == bound_type.Type.DOTTED_WHITE:
                                types.append(LaneBoundType.DOTTED_WHITE)
                            elif type == bound_type.Type.SOLID_YELLOW:
                                types.append(LaneBoundType.SOLID_YELLOW)
                            elif type == bound_type.Type.SOLID_WHITE:
                                types.append(LaneBoundType.SOLID_WHITE)
                            elif type == bound_type.Type.DOUBLE_YELLOW:
                                types.append(LaneBoundType.DOUBLE_YELLOW)
                            elif type == bound_type.Type.CURB:
                                types.append(LaneBoundType.CURB)
                        types = set(types)
                        if side == 'l':
                            left_type = types
                            side = 'r'
                        else:
                            right_type = types
            return BoundTypeInfo(left_type, right_type)

                                
        # @check_type
        def get_sl_by_xy(self, point:Vector, check_bound=True):
            result:Union[SLPoint, None] = None
            
            k_epsilon = 0.1
            pre_segment = None
            s = -k_epsilon
            for nth, segment in enumerate(self.central_segments):
                # add epsilon for the first and the last segment.
                if nth == 0:
                    segment = Segment(segment.p1 - k_epsilon * segment.vec.unit, segment.p2)
                if nth == len(self.central_segments) - 1:
                    segment = Segment(segment.p1, segment.p2 + k_epsilon * segment.vec.unit)
                
                segment_sl = segment.get_sl_by_xy(point)
                
                # check whether point is in the corner of previous segment and current segment.
                if segment_sl is None and not pre_segment is None and \
                    (
                        Segment(pre_segment.p2, pre_segment.p2 + pre_segment.vec.left_unit).to_left_test(point) ^ \
                        Segment(segment.p1, segment.p1 + segment.vec.left_unit).to_left_test(point)
                    ):
                    l = pre_segment.p2.dis_to(point)
                    if pre_segment.to_left_test(segment.p2):
                        l = -l
                    segment_sl = SLPoint(s, l)

                # update result
                if not segment_sl is None:
                    if result is None:
                        result = segment_sl
                        result.s += s
                    elif result.d > segment_sl.d:
                        result = segment_sl
                        result.s += s

                pre_segment = segment
                s += segment.length
            
            # prevent overflow
            if not result is None:
                result.s = clamp(result.s, 1e-6, self.length - 1e-6)
                result.obj = self

                if check_bound:
                    pre_s = 0
                    pre_bound = self.bound[0]
                    for s, bound in self.bound.items():
                        if s > result.s:
                            rate = (result.s - pre_s) / (s - pre_s)
                            min_l = lerp(pre_bound.min_l, bound.min_l, rate)
                            max_l = lerp(pre_bound.max_l, bound.max_l, rate)
                            if result.l < min_l or result.l > max_l:
                                return None
                        pre_s = s
                        pre_bound = bound
            return result
        
        @property
        def length(self):
            ret = 0
            for segment in self.central_segments:
                ret += segment.length
            return ret

        def __repr__(self):
            return f'<Lane : {self.id}>'
        
        def __str__(self):
            return self.__repr__()

        def __eq__(self, lane):
            return self.id == lane.id
        
        def __hash__(self) -> int:
            return int(''.join(filter(lambda ch: ch in '1234567890', self.id)))

class Overlap:
    pass

class CrosswalkLaneOverlap(Overlap):
    def __init__(self, crosswalk, lane, start_s, end_s):
        self._crosswalk = crosswalk
        self._lane = lane
        self.start_s = start_s
        self.end_s = end_s

    @property
    def crosswalk(self):
        return self._crosswalk
    
    @property
    def lane(self):
        return self._lane
    
    @property
    def cover_range(self):
        CoverRange = namedtuple('CoverRange', ['start_s', 'end_s'])
        return CoverRange(self.start_s, self.end_s)

class TrafficLightLaneOverlap(Overlap):
    def __init__(self, traffic_light, lane):
        self._traffic_light = traffic_light
        self._lane = lane

    @property
    def traffic_light(self):
        return self._traffic_light
    
    @property
    def lane(self):
        return self._lane

class Map:    
    @check_type
    def __init__(self, sim_type, map_name:str):
        self.map = MapParser()
        if isinstance(sim_type, str):
            sim_type = simulator.SimulatorType.__members__[sim_type]

        with open(map_config.sim_map_path(sim_type, map_name), 'rb') as f:
            self.map.parse_from_bin(f.read())
        
        self.objects = DictObj()

        # init lanes
        self.lanes = DictObj()
        for lane_proto in self.map.lane:
            self.lanes[lane_proto.id.id] = Lane(self, lane_proto)
            self.objects[lane_proto.id.id] = self.lanes[lane_proto.id.id]

        # init crosswalks
        self.crosswalks = DictObj()
        for crosswalk_proto in self.map.crosswalk:
            if crosswalk_proto.id.id != '':
                self.crosswalks[crosswalk_proto.id.id] = Crosswalk(self, crosswalk_proto)
                self.objects[crosswalk_proto.id.id] = self.crosswalks[crosswalk_proto.id.id]

        # init traffic lights
        self.traffic_lights = DictObj()
        for traffic_light_proto in self.map.signal:
            self.traffic_lights[traffic_light_proto.id.id] = TrafficLight(self, traffic_light_proto)
            self.objects[traffic_light_proto.id.id] = self.traffic_lights[traffic_light_proto.id.id]

        # init overlaps
        self.overlaps = defaultdict(dict)
        for overlap_proto in self.map.overlap:
            if overlap_proto.object[0].id.id in self.objects and overlap_proto.object[1].id.id in self.objects:
                obj1 = self.objects[overlap_proto.object[0].id.id]
                obj2 = self.objects[overlap_proto.object[1].id.id]
                
                cw_lane = Match((obj1, obj2), (Crosswalk, Lane))
                if not cw_lane is None:
                    if cw_lane[0].id == overlap_proto.object[0].id.id:
                        start_s = overlap_proto.object[0].lane_overlap_info.start_s
                        end_s = overlap_proto.object[0].lane_overlap_info.start_s
                    else:
                        start_s = overlap_proto.object[1].lane_overlap_info.start_s
                        end_s = overlap_proto.object[1].lane_overlap_info.start_s
                    self.overlaps[cw_lane[0]][cw_lane[1]] = CrosswalkLaneOverlap(cw_lane[0], cw_lane[1], start_s, end_s)
                    self.overlaps[cw_lane[1]][cw_lane[0]] = CrosswalkLaneOverlap(cw_lane[0], cw_lane[1], start_s, end_s)
                    continue
                
                tl_lane = Match((obj1, obj2), (TrafficLight, Lane))
                if not tl_lane is None:
                    self.overlaps[tl_lane[0]][tl_lane[1]] = TrafficLightLaneOverlap(tl_lane[0], tl_lane[1])
                    self.overlaps[tl_lane[1]][tl_lane[0]] = TrafficLightLaneOverlap(tl_lane[0], tl_lane[1])

        self.kdtree = self._construct_kdtree()
        self.visualizer = MapVisualizerServer(self)

    def _construct_kdtree(self):
        objects = []
        for id, lane in self.lanes.items():
            s = 0
            for segment in lane.central_segments:
                segment_info = SegmentInfo(segment, lane, s)
                objects.append(segment_info)
                s += segment.length
        return KDTree(objects)

    def get_lane_by_id(self, lane_id):
        assert lane_id in self.lanes, f'<Lane : {lane_id}> is not in this map.'
        return self.lanes[lane_id]

    def match_lanes(self, condition)-> Tuple['Lane']:
        ret = []
        for id, lane in self.lanes.items():
            try:
                if condition(lane):
                    ret.append(lane)
            except:
                pass
        return tuple(ret)


    # @check_type
    def get_lane_info_by_xy(self, point:Vector, pre_lane = None, post_lane = None):
        ref_lanes = set()
        if not pre_lane is None and not post_lane is None:
            ref_lanes = pre_lane.neighbors & post_lane.neighbors
            ref_lanes.add(pre_lane)
            ref_lanes.add(post_lane)
        else:
            if not pre_lane is None:
                ref_lanes = pre_lane.neighbors
                ref_lanes.add(pre_lane)
            elif not post_lane is None:
                ref_lanes = post_lane.neighbors
                ref_lanes.add(post_lane)
        
        result = None
        for lane in ref_lanes:
            sl_point = lane.get_sl_by_xy(point)
            if not sl_point is None:
                if result is None or result.d > sl_point.d:
                    result = sl_point
        
        if result is None:
            return self.get_closest_lane(point)
        else:
            return result

    # @check_type
    def get_closest_lane(self, point, check_bound=True) -> SLPoint:
        if isinstance(point, simulator.AgentState):
            point = point.position

        segment_info = self.kdtree.get_closet_object(point)
        lane = segment_info.lane
        sl_point = lane.get_sl_by_xy(point, check_bound)
        if sl_point is None:
            # lanes = self.get_lanes_by_point_iter(point)
            # if len(lanes) == 0:
            #     raise OutOfRoadError
            # else:
            #     return lanes[0]
            raise OutOfRoadError
        return sl_point


    def get_lanes_by_point_iter(self, point):
        ret = []
        for lane_id, lane in self.lanes.items():
            sl_point = lane.get_sl_by_xy(point)
            if not sl_point is None:
                ret.append(sl_point)
        return tuple(ret)
    
    def get_intersect_points(self, _segment, except_id):
        result = []
        for lane in self.map.lane:
            if lane.id != except_id:
                for points in lane.central_curve.segment[0].line_segment.point:
                    points = [Vector(p.x, p.y) for p in points]
                    for i in range(len(points) - 1):
                        segment = Segment(points[i], points[i+1])
                        cross_point = intersect_point(segment, _segment)
                        if not cross_point is None:
                            result.append(cross_point)
        return result            

    def __getattr__(self, key):
        return self.map.__getattribute__(key)

    def run_visualizer(self):
        self.visualizer.run()

    def plot(self, figsize=(20, 20), mark=[], points=[], traces=[], lane_filter=None):
        plt.figure(figsize=figsize)

        # draw lanes
        default_color = 'b'
        defualt_emphc = 'r'
        marked_lane = dict()
        for mark_info in mark:
            if type(mark_info) == tuple:
                marked_lane[mark_info[0]] = mark_info[1]
            else:
                marked_lane[mark_info] = defualt_emphc
        for lane_id, lane in self.lanes.items():
            if not lane_filter is None and not lane_id in lane_filter:
                continue
            x = [v.x for v in lane.central_points]
            y = [v.y for v in lane.central_points]
            color = marked_lane[lane_id] if lane_id in marked_lane else default_color
            plt.plot(x, y, color=color)
        

        # draw points
        for point in points:
            if type(point) == tuple:
                plt.plot(point[0].x, point[0].y, 'o', color=point[1])
            else:
                plt.plot(point.x, point.y, 'o', color='g')
        
        # draw traces
        for trace in traces:
            if type(trace) == tuple:
                color = trace[1]
                trace = trace[0]
            else:
                color = 'orange'
            for t, s in trace.iter():
                poly = [s.corner_position('fl'), s.corner_position('fr'), s.corner_position('br'), s.corner_position('bl'), s.corner_position('fl')]
                plt.plot([v.x for v in poly], [v.y for v in poly], color=color, linewidth=0.1)

        plt.axis('equal')
        
        plt.show()