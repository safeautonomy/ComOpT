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

import random

import comopt.database as db
import comopt.map_parse as mp
from comopt.geometry import Vector, Segment, check_intersect

crosswalk_map = {}


def init_crosswalk_map():
    global crosswalk_map
    crosswalk_map = {}
    for c in mp.global_map.map.crosswalk:
        crosswalk_map[c.id.id] = c


class Crosswalk:
    def __init__(self, _id, ls):
        self.id = _id
        self.lanes = ls
        self.polygon = []
        self.roadside = None
        for p in crosswalk_map[_id].polygon.point:
            self.polygon.append(Vector(p.x, p.y, p.z))

    def info(self):
        polygon_str = []
        for p in self.polygon:
            polygon_str.append(p.str_3d())
        return {
            'id': self.id,
            'lanes': self.lanes,
            'polygon': polygon_str,
        }

    def get_roadside(self):
        if len(self.polygon) != 4:
            raise NameError('Invalid polygon point count')
        points = self.polygon + [self.polygon[0]]
        roadside = []
        for i in range(4):
            polygon_seg = Segment(points[i], points[i+1])
            intersect = False
            lanes = [
                mp.global_map.get_closest_lane(polygon_seg.p1, False).obj,
                mp.global_map.get_closest_lane(polygon_seg.p2, False).obj,
            ]
            for lane in lanes:
                for lane_seg in lane.central_segments:
                    if check_intersect(polygon_seg, lane_seg):
                        intersect = True
                        break
                if intersect:
                    break
            # print(polygon_seg, lanes, intersect)
            if not intersect:
                roadside.append(polygon_seg)
        if len(roadside) == 3:
            m = {}
            for r in roadside:
                for p in [r.p1, r.p2]:
                    if str(p) not in m.keys():
                        m[str(p)] = 0
                    m[str(p)] += 1
            for i in range(len(roadside)):
                r = roadside[i]
                if m[str(r.p1)] == 2 and m[str(r.p2)] == 2:
                    roadside = roadside[:i]+roadside[i+1:]
                    break
        if len(roadside) != 2:
            raise NameError(f'Unexpected roadside count: {len(roadside)}')
        return roadside

    def get_random_path(self):
        rs = self.get_roadside()
        if random.random() > 0.5:
            return [rs[0].get_random_point(), rs[1].get_random_point()]
        else:
            return [rs[1].get_random_point(), rs[0].get_random_point()]

    def central_point(self):
        ret = Vector()
        for point in self.polygon:
            ret += point
        ret *= 1 / len(self.polygon)
        return ret

    def ratio_of_lane(self, lane_id):
        lane = mp.global_map.lanes[lane_id]
        sl = None
        for p in [self.central_point()] + self.polygon:
            sl = lane.get_sl_by_xy(p, False)
            if sl is not None:
                break
        return sl.s / lane.length


def parse_crosswalks():
    sel = db.Selector()
    sel.open_session()
    ret = sel.with_format(
        '(l:lane)-[:lane_to_crosswalk]->(c:crosswalk)',
        None,
        ['l', 'c']
    )
    lanes = db.List('l', ret)
    crosswalks = db.List('c', ret)
    crosswalk_lane_map = db.list_to_map(crosswalks, lanes)
    lane_crosswalk_map = db.list_to_map(lanes, crosswalks)
    crosswalks = {}
    for c in crosswalk_lane_map.keys():
        crosswalks[c] = Crosswalk(c, list(crosswalk_lane_map[c]))
    for l in lane_crosswalk_map.keys():
        crosswalk_list = list(lane_crosswalk_map[l])
        lane_crosswalk_map[l] = []
        for c in crosswalk_list:
            lane_crosswalk_map[l].append(crosswalks[c])
    sel.close_session()
    sel.close()
    return lane_crosswalk_map


def add_crosswalks_info_in_junctions(junction_list, lane_crosswalk_map):
    for j in junction_list:
        lanes = set()
        for cl in j.central_lanes:
            lanes.add(cl)
        for c in j.connectors.keys():
            lanes = lanes | j.connectors[c].entrance | j.connectors[c].exit
        j.crosswalks = {}
        for ll in list(lanes):
            if ll in lane_crosswalk_map.keys():
                for c in lane_crosswalk_map[ll]:
                    j.crosswalks[c.id] = c
    return junction_list
