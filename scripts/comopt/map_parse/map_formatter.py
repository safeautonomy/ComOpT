#
#    ComOpT: COMbination and OPtimization for Testing autonomous driving systems
#
#    Copyright (C) 2021  Yuhang Chen, Chih-Hong Cheng, Changwen Li, Tiantian Sun, Rongjie Yan
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Affero General Public License as published
#    by the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Affero General Public License for more details.
#
#    You should have received a copy of the GNU Affero General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

from comopt.map_parse import overlap_formatter as oft
from comopt.map_parse.overlap_formatter import format_overlap
from comopt.property_graph.edge import add_edge_appearing_twice
from comopt.property_graph import graph
from loguru import logger as log


class MapFormatter:
    def __init__(self, m):
        self.overlaps = dict()
        self.m = m
        self.g = graph.Graph()
        self.init_overlap()

    # format vertices
    def format_vertices(self):
        self.format_lane_vertices()
        self.format_signal_vertices()
        self.format_stop_sign_vertices()
        self.format_crosswalk_vertices()
        self.format_junction_vertices()
        self.format_road_vertices()

    def format_lane_vertices(self):
        for lane in self.m.map.lane:
            self.g.add_vertex(lane.id.id, {
                'type': 'lane',
                'length': lane.length,
                'speed_limit': lane.speed_limit,
                'lane_type': lane.type,
                'turn': lane.turn,
                'direction': lane.direction,
                'central_curve_': lane.central_curve,
                'left_boundary_': lane.left_boundary,
                'right_boundary_': lane.right_boundary,
                'left_sample_': lane.left_sample,
                'right_sample_': lane.right_sample,
                'left_road_sample_': lane.left_road_sample,
                'right_road_sample_': lane.right_road_sample,
            })

    def format_signal_vertices(self):
        for signal in self.m.map.signal:
            self.g.add_vertex(signal.id.id, {
                'type': 'signal',
                # 'boundary': signal.boundary,
            })
            for s in signal.subsignal:
                self.g.add_vertex(f'{signal.id.id}_subsignal_{s.id.id}', {
                    'type': 'subsignal',
                    'subsignal_type': s.type,
                    'location_': s.location,
                })
                self.g.add_edge(graph.Edge(
                    f'{signal.id.id}_has_subsignal_{s.id.id}', signal.id.id, f'{signal.id.id}_subsignal_{s.id.id}', {
                        'type': 'signal_has_subsignal',
                    }))

    def format_stop_sign_vertices(self):
        for stop_sign in self.m.map.stop_sign:
            self.g.add_vertex(stop_sign.id.id, {
                'type': 'stop_sign',
                'stop_line_': stop_sign.stop_line,
                'stop_sing_type': stop_sign.type,
            })

    def format_crosswalk_vertices(self):
        for crosswalk in self.m.map.crosswalk:
            if crosswalk.id.id != '':  # TODO: consider adding wild crosswalk
                # print(crosswalk.id.id)
                self.g.add_vertex(crosswalk.id.id, {
                    'type': 'crosswalk',
                    'polygon_': crosswalk.polygon,
                })

    def format_junction_vertices(self):
        for junction in self.m.map.junction:
            self.g.add_vertex(junction.id.id, {
                'type': 'junction',
                'polygon_': junction.polygon,
            })

    def format_road_vertices(self):
        for road in self.m.map.road:
            self.g.add_vertex(road.id.id, {
                'type': 'road',
                'road_type': road.type,
            })
            for s in road.section:
                self.g.add_vertex(f'{road.id.id}_section_{s.id.id}', {
                    'type': 'section',
                    'boundary_': s.boundary,
                })
                self.g.add_edge(graph.Edge(
                    f'{road.id.id}_has_section_{s.id.id}', road.id.id, f'{road.id.id}_section_{s.id.id}', {
                        'type': 'road_has_section',
                    }))

    # format edges
    def format_edges(self):
        self.format_lane_edges()
        self.format_stop_sign_edges()
        self.format_crosswalk_edges()
        self.format_junction_edges()
        self.format_road_edges()
        self.format_road_section()

    def init_overlap(self):
        for overlap in self.m.map.overlap:
            self.overlaps[overlap.id.id] = overlap

    def format_overlaps(self, overlap_ids, overlap_formatters):
        for overlap_id in overlap_ids:
            o = self.overlaps[overlap_id.id]
            edge, ok = format_overlap(o, overlap_formatters)
            if not ok:
                log.error(f'Unexpected type of overlap:{overlap_id.id}')
                continue
                # raise NameError(f'Unexpected type of overlap:{o}')
            add_edge_appearing_twice(self.g, edge)

    def format_lane_edges(self):
        for lane in self.m.map.lane:
            _id = lane.id.id
            # successor
            for p in lane.predecessor_id:
                add_edge_appearing_twice(self.g, graph.Edge(f'{p.id}_successor_{_id}', p.id, _id,
                                                            {'type': 'lane_successor_lane'}))
            for s in lane.successor_id:
                add_edge_appearing_twice(self.g, graph.Edge(f'{_id}_successor_{s.id}', _id, s.id,
                                                            {'type': 'lane_successor_lane'}))
            # neighbor_forward
            for p in lane.left_neighbor_forward_lane_id:
                add_edge_appearing_twice(self.g, graph.Edge(f'{p.id}_neighbor_forward_{_id}', p.id, _id,
                                                            {'type': 'lane_neighbor_forward_lane'}))
            for s in lane.right_neighbor_forward_lane_id:
                add_edge_appearing_twice(self.g, graph.Edge(f'{_id}_neighbor_forward_{s.id}', _id, s.id,
                                                            {'type': 'lane_neighbor_forward_lane'}))
            # neighbor_reverse
            for p in lane.left_neighbor_reverse_lane_id:
                add_edge_appearing_twice(self.g, graph.Edge(f'{p.id}_neighbor_reverse_{_id}', p.id, _id,
                                                            {'type': 'lane_neighbor_reverse_lane'}))
            for s in lane.right_neighbor_reverse_lane_id:
                add_edge_appearing_twice(self.g, graph.Edge(f'{_id}_neighbor_reverse_{s.id}', _id, s.id,
                                                            {'type': 'lane_neighbor_reverse_lane'}))

            # overlap
            self.format_overlaps(lane.overlap_id, [
                oft.LaneToSignal(),
                oft.LaneToStopSign(),
                oft.LaneToCrosswalk(),
                oft.JunctionToLane(),
            ])

    def format_stop_sign_edges(self):
        for stop_sign in self.m.map.stop_sign:
            # overlap
            self.format_overlaps(stop_sign.overlap_id, [
                oft.LaneToStopSign(),
                oft.JunctionToStopSign(),
            ])

    def format_crosswalk_edges(self):
        for crosswalk in self.m.map.crosswalk:
            # overlap
            self.format_overlaps(crosswalk.overlap_id, [
                oft.LaneToCrosswalk(),
            ])

    def format_junction_edges(self):
        for junction in self.m.map.junction:
            # overlap
            self.format_overlaps(junction.overlap_id, [
                oft.JunctionToLane(),
                oft.JunctionToStopSign(),
                oft.JunctionToSubSignal(),
            ])

    def format_road_edges(self):
        for road in self.m.map.road:
            if road.junction_id.id == '':
                continue
            edge = graph.Edge(f'{road.junction_id.id}_to_{road.id.id}',
                              road.junction_id.id, road.id.id,
                              {'type': 'junction_to_road'})
            add_edge_appearing_twice(self.g, edge)

    def format_road_section(self):
        for road in self.m.map.road:
            for section in road.section:
                for lane_id in section.lane_id:
                    edge = graph.Edge(f'{road.id.id}_section_{section.id.id}_to_{lane_id.id}',
                                      f'{road.id.id}_section_{section.id.id}', lane_id.id,
                                      {'type': 'section_to_lane'})
                    add_edge_appearing_twice(self.g, edge)
