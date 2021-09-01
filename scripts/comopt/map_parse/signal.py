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

import comopt.database as db
import comopt.map_parse as mp
from comopt.geometry import Vector
from loguru import logger as log

signal_map = {}


def init_signal_map():
    global signal_map
    signal_map = {}
    for s in mp.global_map.map.signal:
        signal_map[s.id.id] = s


class Subsignal:
    def __init__(self, obj):
        self.id = obj.id.id
        self.type = obj.type

    def info(self):
        return {
            'id': self.id,
            'type': self.type
        }


class Signal:
    def __init__(self, _id, ls):
        self.id = _id
        self.lanes = ls
        self.boundary = []
        for p in signal_map[_id].boundary.point:
            self.boundary.append(Vector(p.x, p.y, p.z))
        self.subsignal = []
        for ss in signal_map[_id].subsignal:
            self.subsignal.append(Subsignal(ss))

    def info(self):
        return {
            'id': self.id,
            'lanes': self.lanes,
            'boundary': [b.str_3d() for b in self.boundary],
            'subsignal': [ss.info() for ss in self.subsignal]
        }


def parse_signals():
    sel = db.Selector()
    sel.open_session()
    ret = sel.with_format(
        '(l:lane)-[:lane_to_signal]->(s:signal)',
        None,
        ['l', 's']
    )
    lanes = db.List('l', ret)
    signals = db.List('s', ret)
    lane_signal_map = db.list_to_map(lanes, signals)
    signal_lane_map = db.list_to_map(signals, lanes)
    signals = {}
    for ll in lane_signal_map.keys():
        try:
            if len(lane_signal_map[ll]) > 1:
                log.error('Multiple signals for one lane')
                continue
            s = lane_signal_map[ll].pop()
            signals[ll] = Signal(s, list(signal_lane_map[s]))
        except Exception as e:
            log.error(f'Fail to parse signal {ll}, {e}')
    sel.close_session()
    sel.close()
    return signals


def add_signals_info_in_junctions(junction_list, lane_signal_map):
    for j in junction_list:
        lanes = set()
        for cl in j.central_lanes:
            lanes.add(cl)
        for c in j.connectors.keys():
            lanes = lanes | j.connectors[c].entrance
        j.signals = {}
        for ll in list(lanes):
            if ll in lane_signal_map.keys():
                s = lane_signal_map[ll]
                if s.id not in j.signals.keys():
                    j.signals[s.id] = s
    return junction_list
