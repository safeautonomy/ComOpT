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

from collections import namedtuple
from os import stat_result
from sys import settrace
from typing import List, Tuple

from matplotlib.pyplot import tick_params
from comopt.utils import DictObj, TimeFunction, Or, And
import comopt.map as map
from comopt.analyzer.behavior import BehaviorFactory, BehaviorType, BehaviorAutomata
from enum import Enum

import sys

sys.setrecursionlimit(1000000)

class TraceAnalyzer:
    LaneInfo = namedtuple('LaneInfo', ['center', 'fl', 'fr', 'br', 'bl'])
    def __init__(self, trace, map):
        if trace.time_step < 0.5:
            self._trace = TimeFunction()
            time_step = trace.time_step
            cnt = 0
            for t, s in trace.iter():
                if cnt == 0:
                    self._trace[t] = s
                cnt += time_step
                if cnt > 0.5:
                    cnt = 0
                
        else:
            self._trace = trace # TimeFunction: time -> state
        self._lane_info = None
        self._behavior_info = None
        self._pattern = None
        self.map = map
        self._out_of_road = False


    @property
    def lane_info(self):
        if not self._lane_info is None:
            return self._lane_info
        self._lane_info = TimeFunction()
       
        ticks = self.trace.ticks
        def get_sl_seq_by_pos_seq(pos_seq, cur_tick=0, sl_seq = None):
            if cur_tick == len(ticks):
                return sl_seq
            if cur_tick == 0:
                sl_point = self.map.get_lane_info_by_xy(pos_seq[cur_tick])
                sl_seq = [sl_point]
                return get_sl_seq_by_pos_seq(pos_seq, cur_tick + 1, sl_seq)
            else:
                pre_sl = sl_seq[-1]
                ref_lanes = {pre_sl.obj} | pre_sl.obj.neighbors
                for lane in ref_lanes:
                    sl_point = lane.get_sl_by_xy(pos_seq[cur_tick])
                    if not sl_point is None:
                        sl_seq.append(sl_point)
                        ret = get_sl_seq_by_pos_seq(pos_seq, cur_tick + 1, sl_seq)
                        if not ret is None:
                            return ret
                        sl_seq.pop()
            try:
                sl_point = self.map.get_closest_lane(pos_seq[cur_tick])
            except map.OutOfRoadError:
                self._out_of_road = True
                return sl_seq
            sl_seq.append(sl_point)
            return get_sl_seq_by_pos_seq(pos_seq, cur_tick + 1, sl_seq)

        center_pos_seq = []
        fl_pos_seq = []
        fr_pos_seq = []
        br_pos_seq = []
        bl_pos_seq = []
        for t, state in self.trace.iter():
            center_pos_seq.append(state.position)
            fl_pos_seq.append(state.corner_position('fl'))
            fr_pos_seq.append(state.corner_position('fr'))
            br_pos_seq.append(state.corner_position('br'))
            bl_pos_seq.append(state.corner_position('bl'))
        
        center_sl_seq = get_sl_seq_by_pos_seq(center_pos_seq)
        fl_sl_seq = get_sl_seq_by_pos_seq(fl_pos_seq)
        fr_sl_seq = get_sl_seq_by_pos_seq(fr_pos_seq)
        br_sl_seq = get_sl_seq_by_pos_seq(br_pos_seq)
        bl_sl_seq = get_sl_seq_by_pos_seq(bl_pos_seq)
        
        min_length = float('inf')
        min_length = min(min_length, len(center_sl_seq))
        min_length = min(min_length, len(fl_sl_seq))
        min_length = min(min_length, len(fr_sl_seq))
        min_length = min(min_length, len(br_sl_seq))
        min_length = min(min_length, len(bl_sl_seq))

        for t, center_sl, fl_sl, fr_sl, br_sl, bl_sl in zip(
            ticks[:min_length], 
            center_sl_seq[:min_length], 
            fl_sl_seq[:min_length], 
            fr_sl_seq[:min_length], 
            br_sl_seq[:min_length], 
            bl_sl_seq[:min_length]
        ):
            self._lane_info[t] = self.LaneInfo(
                center = center_sl,
                fl = fl_sl,
                fr = fr_sl,
                br = br_sl,
                bl = bl_sl
            )

        return self._lane_info

    @property
    def out_of_road(self):
        return self._out_of_road

    @property
    def behaviors(self):
        automata = BehaviorAutomata()
        B = automata.input([lane_info for t, lane_info in self.lane_info.iter()])
        ticks = self.lane_info.ticks

        start_step = 0
        step = 1
        self._behavior_info = []
        BehaviorTraceItem = namedtuple('BehaviorTraceItem', ['state', 'lane_info'])
        
        while step < len(ticks):
            while step < len(ticks) and B[step] == B[step - 1]:
                step += 1
            behavior_type = B[start_step]
            
            sub_trace = TimeFunction()

            for i in range(start_step, step):
                t = ticks[i]
                sub_trace[t] = BehaviorTraceItem(state = self._trace[t], lane_info=self.lane_info[t])

            self._behavior_info.append(BehaviorFactory.get_behavior_obj(behavior_type, sub_trace))

            start_step = step
            step += 1
        
        self._behavior_info = tuple(self._behavior_info)
        return self._behavior_info

    @property
    def pattern(self):
        if not self._pattern is None:
            return self._pattern
        pattern = []
        for behavior in self.behaviors:
            pattern.append((behavior.type, behavior.pattern))
        self._pattern = tuple(pattern)
        return self._pattern

    @property
    def trace(self):
        return self._trace
