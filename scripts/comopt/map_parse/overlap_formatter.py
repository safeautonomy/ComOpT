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

import re

from comopt.property_graph import graph


def format_overlap(overlap, formatters):
    ok = False
    ret = None
    for f in formatters:
        f.set(overlap)
        if not f.validate():
            continue
        if ok:
            raise NameError(f'Two formatter fit one overlap:{overlap}')
        ret = f.format()
        ok = True
    return ret, ok


class OverlapFormatter:
    def __init__(self):
        self.overlap = None

    def set(self, o):
        self.overlap = o

    def validate(self):
        pass

    def format(self):
        pass

# TODO: add overlap information storing


class LaneToSignal(OverlapFormatter):
    def validate(self):
        return re.match(r'overlap_signal_\d+_lane_\d+', self.overlap.id.id)

    def format(self):
        nums = re.findall(r'\d+', self.overlap.id.id)
        signal_id = f'signal_{nums[0]}'
        lane_id = f'lane_{nums[1]}'
        return graph.Edge(f'{lane_id}_to_{signal_id}', lane_id, signal_id, {'type': 'lane_to_signal'})


class LaneToStopSign(OverlapFormatter):
    def validate(self):
        return re.match(r'overlap_stopsign_\d+_lane_\d+', self.overlap.id.id)

    def format(self):
        nums = re.findall(r'\d+', self.overlap.id.id)
        stop_sign_id = f'stopsign_{nums[0]}'
        lane_id = f'lane_{nums[1]}'
        return graph.Edge(f'{lane_id}_to_{stop_sign_id}', lane_id, stop_sign_id, {'type': 'lane_to_stop_sign'})


class LaneToCrosswalk(OverlapFormatter):
    def validate(self):
        return re.match(r'overlap_CW_\d+_lane_\d+', self.overlap.id.id)

    def format(self):
        nums = re.findall(r'\d+', self.overlap.id.id)
        crosswalk_id = f'CW_{nums[0]}'
        lane_id = f'lane_{nums[1]}'
        return graph.Edge(f'{lane_id}_to_{crosswalk_id}', lane_id, crosswalk_id, {'type': 'lane_to_crosswalk'})


class JunctionToLane(OverlapFormatter):
    def validate(self):
        return re.match(r'overlap_junction_I\d+_J\d+_lane_\d+', self.overlap.id.id)

    def format(self):
        nums = re.findall(r'\d+', self.overlap.id.id)
        junction_id = f'J_{nums[1]}'
        lane_id = f'lane_{nums[2]}'
        return graph.Edge(f'{junction_id}_to_{lane_id}', junction_id, lane_id, {'type': 'junction_to_lane'})


class JunctionToStopSign(OverlapFormatter):
    def validate(self):
        return re.match(r'overlap_junction_I\d+_J\d+_stopsign\d+', self.overlap.id.id)

    def format(self):
        nums = re.findall(r'\d+', self.overlap.id.id)
        junction_id = f'J_{nums[1]}'
        stop_sign_id = f'stopsign_{nums[2]}'
        # TODO: why duplicate here?
        return graph.Edge(f'{junction_id}_to_{stop_sign_id}_I{nums[0]}', junction_id, stop_sign_id,
                          {'type': 'junction_to_stop_sign'})


class JunctionToSubSignal(OverlapFormatter):
    def validate(self):
        return re.match(r'overlap_junction_I\d+_J\d+_signal_\d+_\d+', self.overlap.id.id)

    def format(self):
        nums = re.findall(r'\d+', self.overlap.id.id)

        junction_id = f'J_{nums[1]}'
        subsignal_id = f'signal_{nums[2]}_subsignal_{nums[3]}'
        return graph.Edge(f'{junction_id}_to_{subsignal_id}_I{nums[0]}', junction_id, subsignal_id,
                          {'type': 'junction_to_subsignal'})

