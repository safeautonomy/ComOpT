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

import json
from loguru import logger as log

import comopt.database as db
from itertools import permutations
import math

from comopt.geometry import Vector


def get_lane_direction(m, lane_id, index):
    l = m.get_lane_by_id(lane_id)
    segment = l.lane_proto.central_curve.segment[0].line_segment.point
    if index < 2 - len(segment) or index > len(segment) - 2:
        raise NameError('Index out of bound')
    index = (index + len(segment) - 1) % (len(segment) - 1)
    p1 = Vector(segment[index].x, segment[index].y, 0)
    p2 = Vector(segment[index + 1].x, segment[index + 1].y, 0)
    return p2 - p1


def get_lane_length(m, lane_id):
    l = m.get_lane_by_id(lane_id)
    return l.lane_proto.length


def neighbor_to_lists(l_from, l_to):
    if l_from.empty() and l_to.empty():
        return []
    heads = []
    for ll in l_from.names:
        if ll not in l_to.names:
            heads.append(ll)
    if len(heads) == 0:
        raise NameError('Could not find head')
    neighbor_map = db.list_to_map(l_from, l_to)
    rets = []
    for head in heads:
        p = head
        ret = []
        while p in neighbor_map.keys():
            ret.append(p)
            p = neighbor_map[p].pop()
        ret.append(p)
        rets.append(ret)
    return rets


# l_from, l_to are db.List
def neighbor_to_list(l_from, l_to):
    head = None
    for ll in l_from.names:
        if ll not in l_to.names:
            head = ll
            break
    if head is None:
        raise NameError('Could not find head')
    neighbor_map = db.list_to_map(l_from, l_to)
    p = head
    ret = []
    while p in neighbor_map.keys():
        ret.append(p)
        p = neighbor_map[p].pop()
    ret.append(p)
    return ret


class Connector:
    def __init__(self):
        self.entrance = set()
        self.exit = set()
        self.entrance_list = None
        self.exit_list = None

    def __lt__(self, other):
        if len(self.entrance) == len(other.entrance):
            return len(self.exit) < len(other.exit)
        return len(self.entrance) < len(other.entrance)

    def __gt__(self, other):
        if len(self.entrance) == len(other.entrance):
            return len(self.exit) > len(other.exit)
        return len(self.entrance) > len(other.entrance)

    def info(self):
        return f"({len(self.entrance)}, {len(self.exit)})"

    def __str__(self):
        return self.info()

    def __repr__(self):
        return self.info()

    def direction(self, m):
        ret = Vector()
        for ent in self.entrance:
            ret += get_lane_direction(m, ent, -1)
        for ext in self.exit:
            ret -= get_lane_direction(m, ext, 0)
        ret /= len(self.entrance) + len(self.exit)
        return ret

    def angle_with(self, other, m):
        return self.direction(m).angle_with_in_xoy(other.direction(m))

    def endpoints(self):
        return {
            'entrance': self.entrance_list,
            'exit': self.exit_list
        }

    # sel a selector, with open session
    def sort_endpoints(self, sel):
        ret = sel.with_formats([
            '(l_ent_from:lane)-[:lane_neighbor_forward_lane]->(l_ent_to:lane)',
            '(l_ext_from:lane)-[:lane_neighbor_forward_lane]->(l_ext_to:lane)',
            # '(lr:lane)-[:lane_neighbor_reverse_lane]->(:lane)',
        ], [
            True,
            True,
            # True
        ], [
            {
                'l_ent_from': {'name': list(self.entrance)},
                'l_ent_to': {'name': list(self.entrance)}
            },
            {
                'l_ext_from': {'name': list(self.exit)},
                'l_ext_to': {'name': list(self.exit)}
            },
            # {
            #     'lr': {'name': list(self.entrance) + list(self.exit)}
            # },
        ], [
            'l_ent_from', 'l_ent_to',
            'l_ext_from', 'l_ext_to',
        ])
        l_ent_from = db.List('l_ent_from', ret)
        l_ent_to = db.List('l_ent_to', ret)
        l_ext_from = db.List('l_ext_from', ret)
        l_ext_to = db.List('l_ext_to', ret)
        if not (l_ent_from.empty() or l_ent_to.empty()):
            self.entrance_list = neighbor_to_list(l_ent_from, l_ent_to)
        elif len(self.entrance) == 1:
            self.entrance_list = list(self.entrance)
        elif len(self.entrance) == 0:
            self.entrance_list = []
        else:
            self.entrance_list = list(self.entrance)
            log.error(f'Unexpected relations of entrances, {len(self.entrance)}')
        if not (l_ext_from.empty() or l_ext_to.empty()):
            self.exit_list = neighbor_to_list(l_ext_from, l_ext_to)
        elif len(self.exit) == 1:
            self.exit_list = list(self.exit)
        elif len(self.exit) == 0:
            self.exit_list = []
        else:
            self.exit_list = list(self.exit)
            log.error(f'Unexpected relations of exits, {len(self.exit)}')

    def reverse(self):
        c = Connector()
        c.entrance, c.exit = self.exit, self.entrance
        c.entrance_list, c.exit_list = self.exit_list, self.entrance_list
        return c

    def length(self, m):
        ret = 0
        for endpoint in self.entrance | self.exit:
            ret += get_lane_length(m, endpoint)
        ret /= len(self.entrance) + len(self.exit)
        return ret


class CentralLane:
    def __init__(self, _id):
        self.id = _id
        self.entrance = None
        self.exit = None

    def info(self):
        return {
            'from': self.entrance,
            'self': self.id,
            'to': self.exit
        }


class Junction:
    def __init__(self, _id, central_lanes, connectors):
        self.id = _id
        self.central_lanes = central_lanes
        self.connectors = connectors
        self.crosswalks = None
        self.signals = None
        self.sort_endpoints()

    def info(self):
        ret = ''
        for k in self.connectors.keys():
            ep = self.connectors[k]
            ret += f' {ep.info()}'
        return f'{{{ret} }}'

    def sort_endpoints(self):
        self.connectors = dict(sorted(self.connectors.items(), key=lambda item: item[1]))

    # make sure sorted endpoints before using this
    def __lt__(self, other):
        if len(self.connectors) < len(other.connectors):
            return True
        if len(self.connectors) > len(other.connectors):
            return False
        self_keys = list(self.connectors.keys())
        other_keys = list(other.connectors.keys())
        for i in range(len(self.connectors)):
            if self.connectors[self_keys[i]] < other.connectors[other_keys[i]]:
                return True
            if self.connectors[self_keys[i]] > other.connectors[other_keys[i]]:
                return False
        return False

    def get_signals(self, begin_lane, end_lane):
        search_area = [begin_lane]
        result = []
        for cl in self.central_lanes:
            central_lane = self.central_lanes[cl]
            if central_lane.entrance == begin_lane and central_lane.exit == end_lane:
                search_area.append(central_lane.id)
        # print('search_area', search_area)
        for signal_id in self.signals:
            signal = self.signals[signal_id]
            other_signals = []
            for sig in self.signals:
                if sig != signal_id:
                    other_signals.append(self.signals[sig])
            for lane in search_area:
                if lane in signal.lanes:
                    # print('found signal', signal_id)
                    result.append({
                        'self': signal.boundary[0],
                        'related': [ss.boundary[0] for ss in other_signals]
                    })
        return result

    def get_crosswalk_random_path(self):
        return [self.crosswalks[cw].get_random_path() for cw in self.crosswalks]

    def __str__(self):
        return f'Junction({self.id}) {self.info()}'

    def __repr__(self):
        return f'Junction({self.id}) {self.info()}'


def parse_junction(name):
    sel = db.Selector()
    sel.open_session()

    # find junction lanes
    ret_junction_lane = sel.with_format('(j:junction) --> (r:road) --> (s:section) --> (l:lane)', {
        'j': {
            'name': name,
        },
    }, ['j', 'l'])
    central_lane = db.List('l', ret_junction_lane)
    central_lane_map = dict()
    for cl in central_lane.names:
        central_lane_map[cl] = CentralLane(cl)

    # find junction endpoints
    ret_jep_from = sel.with_format(
        '(from_lane:lane) -[r:lane_successor_lane]-> (central_lane:lane), (from_section:section) --> (from_lane)', {
            'central_lane': central_lane.limitation(),
        }, ['central_lane', 'from_lane', 'from_section'])
    from_central_lane = db.List('central_lane', ret_jep_from)
    from_lane = db.List('from_lane', ret_jep_from)
    from_section = db.List('from_section', ret_jep_from)
    from_map = db.list_to_map(from_central_lane, from_lane)
    from_section_map = db.list_to_map(from_section, from_lane)
    for cl in from_map:
        central_lane_map[cl].entrance = from_map[cl].pop()

    ret_jep_to = sel.with_format(
        '(central_lane:lane) -[r:lane_successor_lane]-> (to_lane:lane), (to_section:section) --> (to_lane)', {
            'central_lane': central_lane.limitation(),
        }, ['central_lane', 'to_lane', 'to_section'])
    to_central_lane = db.List('central_lane', ret_jep_to)
    to_lane = db.List('to_lane', ret_jep_to)
    to_section = db.List('to_section', ret_jep_to)
    to_map = db.list_to_map(to_central_lane, to_lane)
    to_section_map = db.list_to_map(to_section, to_lane)
    for cl in to_map:
        central_lane_map[cl].exit = to_map[cl].pop()

    connectors = dict()
    for s in from_section_map.keys():
        if s not in connectors:
            connectors[s] = Connector()
        connectors[s].entrance = connectors[s].entrance | from_section_map[s]
    for s in to_section_map.keys():
        if s not in connectors:
            connectors[s] = Connector()
        connectors[s].exit = connectors[s].exit | to_section_map[s]

    sel.close_session()
    return Junction(name, central_lane_map, connectors)


def parse_sections_as_junctions():
    sel = db.Selector()
    sel.open_session()
    ret_section_lane = sel.with_format(
        '(r:road)-[:road_has_section]->(s:section)-[:section_to_lane]->(l:lane)',
        'NOT EXISTS((:junction)-[:junction_to_road]->(r))',
        ['s', 'l']
    )
    sections = db.List('s', ret_section_lane)
    lanes = db.List('l', ret_section_lane)
    section_lane_map = db.list_to_map(sections, lanes)
    junctions = []
    for s in section_lane_map.keys():
        ls = section_lane_map[s]
        ret_neighbor = sel.with_format(
            '(l_from:lane)-[:lane_neighbor_forward_lane]->(l_to:lane)',
            {
                'l_from': {'name': list(ls)},
                'l_to': {'name': list(ls)},
            },
            ['l_from', 'l_to'],
        )
        l_from = db.List('l_from', ret_neighbor)
        l_to = db.List('l_to', ret_neighbor)
        # print('l_from and l_to: ', s, l_from.names, l_to.names)
        independent_lanes = []
        for ll in ls:
            if ll not in l_from.names and ll not in l_to.names:
                independent_lanes.append(ll)
        try:
            rets = neighbor_to_lists(l_from, l_to)
            for ll in independent_lanes:
                rets.append([ll])
            if len(rets) == 2:
                c = Connector()
                c.entrance, c.exit = set(rets[0]), set(rets[1])
                c.entrance_list, c.exit_list = rets[0], rets[1]
                connectors = {0: c, 1: c.reverse()}
            elif len(rets) == 1:
                c = Connector()
                c.entrance, c.exit = set(rets[0]), set()
                c.entrance_list, c.exit_list = rets[0], []
                connectors = {0: c, 1: c.reverse()}
            elif len(rets) == 0:
                connectors = {}
            else:
                raise NameError('Unexpected count of lists')
        except Exception as e:
            log.error(f'Fail to parse section {s}: {e}, {ls}, {l_from.names}, {l_to.names}')
            connectors = {}

        j = Junction(s, {}, connectors)
        junctions.append(j)
    sel.close_session()
    sel.close()
    return junctions
