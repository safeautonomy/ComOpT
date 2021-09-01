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
from loguru import logger as log


class AbstractSection:
    def __init__(self, entrance_count, exit_count):
        self.entrance_count = entrance_count
        self.exit_count = exit_count
        self.entrance_permutation = list(permutations(range(0, self.entrance_count)))
        self.exit_permutation = list(permutations(range(0, self.exit_count)))
        ent = len(self.entrance_permutation)
        if ent == 0:
            ent = 1
        ext = len(self.exit_permutation)
        if ext == 0:
            ext = 1
        self.permutation_count = ent * ext

    def __eq__(self, other):
        return self.entrance_count == other.entrance_count and self.exit_count == other.exit_count

    def __lt__(self, other):
        if self.entrance_count == other.entrance_count:
            return self.exit_count < other.exit_count
        return self.entrance_count < other.entrance_count

    def info(self):
        return f'({self.entrance_count},{self.exit_count})'

    def permutation(self, index):
        ext_per_len = len(self.exit_permutation)
        ent = list(self.entrance_permutation[int(index / ext_per_len)])
        ext = list(self.exit_permutation[int(index % ext_per_len)])
        for i in range(0, len(ext)):
            ext[i] += self.entrance_count
        return ent + ext

    def endpoint_count(self):
        return self.entrance_count + self.exit_count


class AbstractCentralLane:
    def __init__(self, _entrance, _exit):
        self.entrance = _entrance
        self.exit = _exit

    def __eq__(self, other):
        return self.entrance == other.entrance and self.exit == other.exit

    def __lt__(self, other):
        if self.entrance == other.entrance:
            return self.exit < other.exit
        return self.entrance < other.entrance


class AbstractJunction:
    def __init__(self, sections, central_lanes):
        self.id = ''
        self.sections = sections
        self.central_lanes = central_lanes
        self.permutation_index = 0
        self.permutation_count = 1
        self.section_inner_permutation_count = 1
        self.section_map = dict()
        for s in sections:
            if s.info() not in self.section_map.keys():
                self.section_map[s.info()] = list()
            self.section_map[s.info()].append(s)
            self.permutation_count *= s.permutation_count
            self.section_inner_permutation_count *= s.permutation_count
        for k in self.section_map:
            ll = range(0, len(self.section_map[k]))
            for i in ll:
                i *= self.section_map[k][0].permutation_count
            self.section_map[k] = {
                'section': self.section_map[k][0],
                'count': len(self.section_map[k]),
                'permutations': list(permutations(ll)),
            }
            self.permutation_count *= len(self.section_map[k])

    def __str__(self):
        return self.id

    def __repr__(self):
        return self.id

    def sort(self):
        self.sections.sort()
        self.central_lanes.sort()

    def permutation(self, index):
        inner_index = index % self.section_inner_permutation_count
        inter_index = index / self.section_inner_permutation_count

        section_list = []
        start = 0
        for k in self.section_map:
            i = inter_index % len(self.section_map[k]['permutations'])
            for j in range(0, self.section_map[k]['count']):
                section_list.append({
                    'start': start + self.section_map[k]['permutations'][int(i)][j] * self.section_map[k][
                        'section'].endpoint_count(),
                    'section': self.section_map[k]['section']
                })
            start += self.section_map[k]['section'].endpoint_count() * self.section_map[k]['count']
            inter_index /= len(self.section_map[k])

        ret_list = []
        for s in section_list:
            ll = s['section'].permutation(inner_index % s['section'].permutation_count)
            for i in ll:
                ret_list.append(i + s['start'])
            inner_index /= s['section'].permutation_count
        return ret_list

    def central_lanes_translate(self, dictionary):
        ret = list()
        for cl in self.central_lanes:
            ret.append(f'{dictionary[cl.entrance]}_{dictionary[cl.exit]}')
        ret.sort()
        ret = tuple(ret)
        return ret

    def central_lanes_without_translation(self):
        ret = list()
        for cl in self.central_lanes:
            ret.append(f'{cl.entrance}_{cl.exit}')
        ret.sort()
        ret = tuple(ret)
        return ret

    def batch_compare(self, dicts, abstract_junction_list):
        # print(f'batch_compare, len = 1 + {len(abstract_junction_list)}')
        if len(abstract_junction_list) == 0:
            return list(), list()
        converted_lanes = list()
        for dictionary in dicts:
            converted_lanes.append(self.central_lanes_translate(dictionary))
        same = list()
        different = list()
        for abj in abstract_junction_list:
            # print('try same: ', end='')
            is_same = False
            # print(abj.central_lanes_without_translation())
            h = hash(abj.central_lanes_without_translation())
            for cls in converted_lanes:
                if hash(cls) == h:
                    # print('  success!', cls)
                    same.append(abj)
                    is_same = True
                    break
            if not is_same:
                # print('  failed.')
                different.append(abj)
        # print(f'same={len(same)}, different={len(different)}')
        return same, different

    def section_info(self):
        self.sort()
        ret = '{'
        for s in self.sections:
            ret += f'{s.info()},'
        ret += '}'
        return ret


def abstract_junction(junction):
    convert_dict = dict()
    abstract_sections = []
    central_lanes = []
    _id = 0
    for k in junction.connectors:
        s = junction.connectors[k]
        for ent in s.entrance:
            convert_dict[ent] = _id
            _id += 1
        for ext in s.exit:
            convert_dict[ext] = _id
            _id += 1
        abstract_sections.append(AbstractSection(len(s.entrance), len(s.exit)))
    for k in junction.central_lanes:
        cl = junction.central_lanes[k]
        if cl.entrance is None or cl.exit is None:
            log.error(f'Unqualified central lane: {cl.entrance} -> {cl.exit}')
            continue
        central_lanes.append(AbstractCentralLane(convert_dict[cl.entrance], convert_dict[cl.exit]))
    aj = AbstractJunction(abstract_sections, central_lanes)
    aj.sort()
    aj.id = junction.id
    return aj


def convert_list_to_dict(convert_list):
    ret = dict()
    for i in range(0, len(convert_list)):
        ret[i] = convert_list[i]
    return ret


def cluster(abstract_junctions):
    if len(abstract_junctions) == 1:
        return [abstract_junctions]
    # log.info(f'cluster check({len(abstract_junctions)}): {abstract_junctions[0].section_info()}')
    dicts = list()
    abj = abstract_junctions[0]
    for i in range(0, abj.permutation_count):
        dicts.append(convert_list_to_dict(abj.permutation(i)))
    ret = list()
    different = abstract_junctions
    while len(different) > 0:
        abj = different[0]
        same, different = abj.batch_compare(dicts, different[1:])
        # print(f'same={len(same)}, different={len(different)}')
        ret.append([abj] + same)
    # log.info(f'cluster check finished({len(abstract_junctions)}):{ret}')
    return ret


def two_tier_cluster(abstract_junctions):
    abj_map = dict()
    # print('start first tier: info cluster')
    for abj in abstract_junctions:
        si = abj.section_info()
        if si not in abj_map.keys():
            abj_map[si] = list()
        abj_map[si].append(abj)
    # print('finish first tier, start second tier')
    for si in abj_map.keys():
        abj_map[si] = cluster(abj_map[si])
    # print('cluster finished')
    return abj_map
