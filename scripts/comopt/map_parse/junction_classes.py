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

from itertools import permutations
from math import pi

import comopt.map_parse as mp

long_section_min_length = 40


def connector_permutations(j):
    cl = [j.connectors[k] for k in j.connectors.keys()]
    return list(permutations(cl))
    # connector_map = dict()
    # for c in j.connectors:
    #     ci = j.connectors[c].info()
    #     if ci not in connector_map.keys():
    #         connector_map[ci] = list()
    #     connector_map[ci].append(j.connectors[c])
    # permutation_list = []
    # for k in connector_map.keys():
    #     permutation_list.append(permutations(connector_map[k]))
    # ret = list()
    # for ll in product(*permutation_list):
    #     rr = list()
    #     for r in ll:
    #         rr += r
    #     ret.append(rr)
    # return ret


def try_junction_classes(j, junction_classes):
    for jc in junction_classes:
        if jc.check(j):
            return jc


class JunctionClass:
    def __init__(self, j=None):
        self.junction = j

    def check(self, j):
        self.junction = j
        return True

    def name(self):
        return 'junction_class'

    def important_lanes(self):
        il = self._important_lanes()
        for case in il.keys():
            for direction in il[case].keys():
                crosswalk_set = set()
                for k in il[case][direction].keys():
                    for lane in il[case][direction][k]:
                        for cw in self.junction.crosswalks.keys():
                            crosswalk = self.junction.crosswalks[cw]
                            if lane in crosswalk.lanes:
                                r = crosswalk.ratio_of_lane(lane)
                                if (r >= 0.5 and k == 'entrance') or (r <= 0.5 and k == 'exit'):
                                    crosswalk_set.add(cw)
                for cl in self.junction.central_lanes:
                    central_lane = self.junction.central_lanes[cl]
                    for cw in self.junction.crosswalks.keys():
                        crosswalk = self.junction.crosswalks[cw]
                        if cl in crosswalk.lanes:
                            r = crosswalk.ratio_of_lane(cl)
                            if central_lane.entrance in il[case][direction]['entrance'] and r <= 0.5:
                                crosswalk_set.add(cw)
                            if central_lane.exit in il[case][direction]['exit'] and r >= 0.5:
                                crosswalk_set.add(cw)
                il[case][direction]['crosswalks'] = list(crosswalk_set)
        return il

    def _important_lanes(self):
        return dict()

    def info(self):
        ret = {
            'id': self.junction.id,
            'crosswalks': [self.junction.crosswalks[c].info() for c in self.junction.crosswalks.keys()],
            'signals': [self.junction.signals[s].info() for s in self.junction.signals.keys()],
            'central_lanes': [self.junction.central_lanes[cl].info() for cl in self.junction.central_lanes.keys()],
        }
        ls = self._important_lanes()
        for k in ls.keys():
            ret[k] = ls[k]
        return ret

    def get_signals(self, begin_lane, end_lane):
        return self.junction.get_signals(begin_lane, end_lane)

    def get_crosswalk_random_path(self):
        return self.junction.get_crosswalk_random_path()

    def __repr__(self):
        return self.name() + f'({self.junction.id})' + self.junction.info()


class J4ends(JunctionClass):
    def check(self, j):
        self.junction = j
        return len(j.connectors) == 4

    def name(self):
        return 'j_4ends'


class J3ends(JunctionClass):
    def check(self, j):
        self.junction = j
        return len(j.connectors) == 3

    def name(self):
        return 'j_3ends'


class J2ends(JunctionClass):
    def check(self, j):
        self.junction = j
        return len(j.connectors) == 2

    def name(self):
        return 'j_2ends'


class JCrossroads(J4ends):
    def __init__(self, j=None):
        super().__init__(j)
        self.c0 = None
        self.c1 = None
        self.c2 = None
        self.c3 = None

    def name(self):
        return 'j_crossroads'

    def check(self, j):
        if not super().check(j):
            return False
        for p in connector_permutations(j):
            c0, c1, c2, c3 = p
            a01 = c0.angle_with(c1, mp.global_map)
            a12 = c1.angle_with(c2, mp.global_map)
            a23 = c2.angle_with(c3, mp.global_map)
            a30 = c3.angle_with(c0, mp.global_map)
            ok = True
            for a in [a01, a12, a23, a30]:
                if pi / 4 < a < 3 * pi / 4:
                    continue
                ok = False
                break
            if ok:
                self.c0 = c0
                self.c1 = c1
                self.c2 = c2
                self.c3 = c3
                self.junction = j
                return True
        return False

    def _important_lanes(self):
        return {
            'case_0': {
                'self': self.c0.endpoints(),
                'right': self.c1.endpoints(),
                'forward': self.c2.endpoints(),
                'left': self.c3.endpoints(),
            },
            'case_1': {
                'self': self.c1.endpoints(),
                'right': self.c2.endpoints(),
                'forward': self.c3.endpoints(),
                'left': self.c0.endpoints(),
            },
            'case_2': {
                'self': self.c2.endpoints(),
                'right': self.c3.endpoints(),
                'forward': self.c0.endpoints(),
                'left': self.c1.endpoints(),
            },
            'case_3': {
                'self': self.c3.endpoints(),
                'right': self.c0.endpoints(),
                'forward': self.c1.endpoints(),
                'left': self.c2.endpoints(),
            }
        }


class JYShaped(J3ends):

    def __init__(self, j=None):
        super().__init__(j)
        self.c0 = None
        self.c1 = None
        self.c2 = None

    def name(self):
        return 'j_y_shaped'

    def check(self, j):
        if not super().check(j):
            return False
        for p in connector_permutations(j):
            c0, c1, c2 = p
            a01 = c0.angle_with(c1, mp.global_map)
            a12 = c1.angle_with(c2, mp.global_map)
            a20 = c2.angle_with(c0, mp.global_map)
            ok = True
            for a in [a01, a12, a20]:
                if pi / 2 < a < 5 * pi / 6:
                    continue
                ok = False
                break
            if ok:
                self.c0 = c0
                self.c1 = c1
                self.c2 = c2
                self.junction = j
                return True
        return False

    def _important_lanes(self):
        return {
            'case_0': {
                'self': self.c0.endpoints(),
                'right_forward': self.c1.endpoints(),
                'left_forward': self.c2.endpoints(),
            },
            'case_1': {
                'self': self.c1.endpoints(),
                'right_forward': self.c2.endpoints(),
                'left_forward': self.c0.endpoints(),
            },
            'case_2': {
                'self': self.c2.endpoints(),
                'right_forward': self.c0.endpoints(),
                'left_forward': self.c1.endpoints(),
            }
        }


class JTShaped(J3ends):

    def __init__(self, j=None):
        super().__init__(j)
        self.c0 = None
        self.c1 = None
        self.c2 = None

    def name(self):
        return 'j_t_shaped'

    def check(self, j):
        if not super().check(j):
            return False
        for p in connector_permutations(j):
            c0, c1, c2 = p
            a01 = c0.angle_with(c1, mp.global_map)
            a12 = c1.angle_with(c2, mp.global_map)
            a20 = c2.angle_with(c0, mp.global_map)
            ok = True
            for a in [a01, a20]:
                if pi / 4 < a < 3 * pi / 4:
                    continue
                ok = False
                break
            if ok and 5 * pi / 6 < abs(a12) < 7 * pi / 6:
                self.c0 = c0
                self.c1 = c1
                self.c2 = c2
                self.junction = j
                return True
        return False

    def _important_lanes(self):
        return {
            'case_no_forward': {
                'self': self.c0.endpoints(),
                'right': self.c1.endpoints(),
                'left': self.c2.endpoints(),
            },
            'case_no_left': {
                'self': self.c2.endpoints(),
                'right': self.c0.endpoints(),
                'forward': self.c1.endpoints(),
            },
            'case_no_right': {
                'self': self.c1.endpoints(),
                'left': self.c0.endpoints(),
                'forward': self.c2.endpoints(),
            }
        }


class J2EndsStraight(J2ends):
    def __init__(self, j=None):
        super().__init__(j)
        self.c0 = None
        self.c1 = None

    def name(self):
        return 'j_2ends_straight'

    def check(self, j):
        if not super().check(j):
            return False
        for p in connector_permutations(j):
            c0, c1 = p
            a01 = c0.angle_with(c1, mp.global_map)
            if 5 * pi / 6 < abs(a01) < 7 * pi / 6:
                self.c0 = c0
                self.c1 = c1
                self.junction = j
                return True
        return False

    def _important_lanes(self):
        return {
            'case_0': {
                'self': self.c0.endpoints(),
                'other': self.c1.endpoints(),
            },
            'case_1': {
                'self': self.c1.endpoints(),
                'other': self.c0.endpoints(),
            }
        }


class J2EndsLong(J2ends):

    def __init__(self, j=None):
        super().__init__(j)
        self.c0 = None
        self.c1 = None

    def name(self):
        return 'j_2ends_long'

    def check(self, j):
        if not super().check(j):
            return False
        for p in connector_permutations(j):
            c0, c1 = p
            if c0.length(mp.global_map) > long_section_min_length:
                self.c0 = c0
                self.c1 = c1
                self.junction = j
                return True
        return False

    def _important_lanes(self):
        return {
            'case_0': {
                'self': self.c0.endpoints(),
                'other': self.c1.endpoints(),
            },
            'case_1': {
                'self': self.c1.endpoints(),
                'other': self.c0.endpoints(),
            }
        }


class J2EndsLongStraight(J2ends):
    def __init__(self, j=None):
        super().__init__(j)
        self.c0 = None
        self.c1 = None

    def name(self):
        return 'j_2ends_long_straight'

    def check(self, j):
        if not super().check(j):
            return False
        for p in connector_permutations(j):
            c0, c1 = p
            a01 = c0.angle_with(c1, mp.global_map)
            if 5 * pi / 6 < abs(a01) < 7 * pi / 6 and c0.length(mp.global_map) > long_section_min_length:
                self.c0 = c0
                self.c1 = c1
                self.junction = j
                return True
        return False

    def _important_lanes(self):
        return {
            'case_0': {
                'self': self.c0.endpoints(),
                'other': self.c1.endpoints(),
            },
            'case_1': {
                'self': self.c1.endpoints(),
                'other': self.c0.endpoints(),
            }
        }
