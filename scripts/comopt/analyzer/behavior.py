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

from comopt.geometry import check_intersect
from comopt.utils import DictObj
import comopt.map as map
from enum import Enum
from collections import namedtuple

class BehaviorType(Enum):
    FOLLOW_LANE = 1
    CHANGE_LANE = 2
    OVERTAKING = 3
    TURN_LEFT = 4
    TURN_RIGHT = 5
    U_TURN = 6
    REVERSE_DRIVING = 7
    DEPARTURE = 8
    UNKNOWN = 9
    next_ = 10

Keypoint = namedtuple('Keypoint', ['time', 'item'])

class BehaviorBase:
    def __init__(self, trace):
        self._trace = trace

    @property
    def pattern(self):
        pattern = []
        for t, s in self._trace.iter():
            if len(pattern) == 0:
                pattern.append(s.lane_info.center.obj.id)
            elif pattern[-1] != s.lane_info.center.obj.id:
                pattern.append(s.lane_info.center.obj.id)
        return tuple(pattern)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f'{self.__class__.__name__}( {" -> ".join(self.pattern)} )'

    @property
    def trace(self):
        return self._trace

    @property
    def danger(self):
        return 0

    @property
    def keypoints(self) -> tuple:
        return tuple()

class FollowLaneBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.FOLLOW_LANE

    @property
    def keypoints(self):
        timer = 0
        temp = []
        time_step = self._trace.time_step
        for t, b_state in self._trace.iter():
            if timer >= 3:
                timer = 0
                temp.append((t, b_state))
            timer += time_step

        temp.sort(key=lambda x : -x[1].state.velocity.magnitude())

        keypoints = [Keypoint(t, state.lane_info.center) for t, state in temp][:3]
        return keypoints

    @property
    def danger(self):
        return 1

class ChangeLaneBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.CHANGE_LANE

    @property
    def danger(self):
        return 3

    @property
    def keypoints(self):
        result_t = None
        ticks = self._trace.ticks
        start_lane = self._trace[ticks[0]].lane_info.center.obj
        final_lane = self._trace[ticks[-1]].lane_info.center.obj
        for t, b_state in self._trace.iter():
            if b_state.lane_info.center.obj == final_lane:
                if result_t is None:
                    result_t = t
                elif self._trace[t].lane_info.center.d < self._trace[result_t].lane_info.center.d:
                    result_t = t
        return [Keypoint(result_t, self._trace[result_t].lane_info.center)]

class OvertakingBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.OVERTAKING

    @property
    def danger(self):
        return 4

    @property
    def keypoints(self):
        result_t = None
        ticks = self._trace.ticks
        start_lane = self._trace[ticks[0]].lane_info.center.obj
        final_lane = self._trace[ticks[-1]].lane_info.center.obj
        for t, b_state in self._trace.iter():
            if b_state.lane_info.center.obj == final_lane:
                if result_t is None:
                    result_t = t
                elif self._trace[t].lane_info.center.d < self._trace[result_t].lane_info.center.d:
                    result_t = t
        return [Keypoint(result_t, self._trace[result_t].lane_info.center)]

class TurnLeftBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.TURN_LEFT

    @property
    def danger(self):
        return 2

class TurnRightBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.TURN_RIGHT

    @property
    def keypoints(self) -> tuple:
        return tuple()

    @property
    def danger(self):
        return 2

class UTurnBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.U_TURN

    @property
    def danger(self):
        return 3

class UnknownBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.UNKNOWN

    @property
    def danger(self):
        return 0

class DepartureBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.DEPARTURE

    @property
    def danger(self):
        return 3

    @property
    def keypoints(self) -> tuple:
        result_t = None
        result_lane_info = None
        ticks = self._trace.ticks
        start_lane = self._trace[ticks[0]].lane_info.center.obj
        final_lane = [ 
            getattr(self._trace[ticks[-2]].lane_info, cf) for cf in ['fl', 'fr', 'bl', 'br'] if getattr(self._trace[ticks[-2]].lane_info, cf).obj != start_lane
        ][0].obj
        for t, b_state in self._trace.iter():
            # if b_state.lane_info.center.obj == final_lane:
            #     if result_t is None:
            #         result_t = t
            #     elif self._trace[t].lane_info.center.d < self._trace[result_t].lane_info.center.d:
            #         result_t = t
            min_d = float('inf')
            for lane_info in [b_state.lane_info.fl, b_state.lane_info.fr, b_state.lane_info.bl, b_state.lane_info.br]:
                if lane_info.obj != start_lane and lane_info.d < 1:
                    result_t = t
                    result_lane_info = lane_info
                    return (Keypoint(result_t, result_lane_info), )    
                    # if result_t is None:
                    #     result_t = t
                    #     result_lane_info = lane_info
                    #     min_d = lane_info.d
                    # elif lane_info.d < min_d:
                    #     min_d = lane_info.d
                    #     result_t = t
                    #     result_lane_info = lane_info
        if not result_t is None:
            return (Keypoint(result_t, result_lane_info), )        
        else:
            return tuple()

class ReverseDrivingBehavior(BehaviorBase):
    @property
    def type(self):
        return BehaviorType.REVERSE_DRIVING
    
    @property
    def danger(self):
        return 5



class BehaviorAutomata:
    TransInfo = namedtuple('TransInfo', ['to_state', 'condition', 'output', 'memory_assignment'])
    def __init__(self, start_state=0):
        self.start_state = start_state
        self.state = self.start_state
        self._state_trans = dict()
        self.memory = DictObj()
        self.init()

    def init(self):
        self._state_trans = dict()
        self.reset()
        self._construct()
    
    def reset(self):
        self.state = self.start_state
        self.memory = DictObj()

    def _add_trans(self, from_state, to_state, condition, output, memory_assignment=lambda inputs, memory: {}):
        if not from_state in self._state_trans:
            self._state_trans[from_state] = []

        self._state_trans[from_state].append(self.TransInfo(to_state, condition, output, memory_assignment))

    def _input(self, inputs):
        trans_id = None
        for i, trans_info in enumerate(self._state_trans[self.state]):
            if trans_info.condition(inputs, self.memory):
                assert trans_id is None, f'Multiple transfer conditions are met. input:{inputs}; state: {self.state}'
                trans_id = i

        if trans_id is None:
            self.memory.update({'pre_inputs': inputs})
            self.state = 7
            return BehaviorType.UNKNOWN    
        assert not trans_id is None, f'No transfer condition is met. input:{inputs}; state: {self.state}'

        trans_info = self._state_trans[self.state][trans_id]
        self.memory.update(trans_info.memory_assignment(inputs, self.memory))
        self.memory.update({'pre_inputs': inputs})
        self.state = trans_info.to_state       
        
        return trans_info.output

    def _construct(self):

        def on_lane(inputs):
            lane_set = []
            if not inputs.fr.obj in lane_set: lane_set.append(inputs.fr.obj)
            if not inputs.fl.obj in lane_set: lane_set.append(inputs.fl.obj)
            if not inputs.br.obj in lane_set: lane_set.append(inputs.br.obj)
            if not inputs.bl.obj in lane_set: lane_set.append(inputs.bl.obj)
            
            for l1 in lane_set:
                for l2 in lane_set:
                    if l1 != l2:
                        if not (l1 in l2.predecessors or l2 in l1.predecessors):
                            return False
            return True

        def exception(inputs, memory):
            if inputs.center.obj not in memory.pre_inputs.center.obj.neighbors | {memory.pre_inputs.center.obj}:
                return True
            if inputs.fl.obj not in memory.pre_inputs.fl.obj.neighbors | {memory.pre_inputs.fl.obj}:
                return True
            if inputs.fr.obj not in memory.pre_inputs.fr.obj.neighbors | {memory.pre_inputs.fr.obj}:
                return True
            if inputs.bl.obj not in memory.pre_inputs.bl.obj.neighbors | {memory.pre_inputs.bl.obj}:
                return True
            if inputs.br.obj not in memory.pre_inputs.br.obj.neighbors | {memory.pre_inputs.br.obj}:
                return True
            return False

        self._add_trans(0, 1, lambda inputs, memory: True, BehaviorType.FOLLOW_LANE)
        
        self._add_trans(1, 7, lambda inputs, memory: exception(inputs, memory), BehaviorType.UNKNOWN)
        self._add_trans(2, 7, lambda inputs, memory: exception(inputs, memory), BehaviorType.UNKNOWN)
        self._add_trans(3, 7, lambda inputs, memory: exception(inputs, memory), BehaviorType.UNKNOWN)
        self._add_trans(4, 7, lambda inputs, memory: exception(inputs, memory), BehaviorType.UNKNOWN)
        self._add_trans(5, 7, lambda inputs, memory: exception(inputs, memory), BehaviorType.UNKNOWN)
        self._add_trans(6, 7, lambda inputs, memory: exception(inputs, memory), BehaviorType.UNKNOWN)


        self._add_trans(1, 1, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs) and inputs.center.obj.turn == map.LaneTurn.NO_TURN, BehaviorType.FOLLOW_LANE)
        self._add_trans(1, 2, lambda inputs, memory: not exception(inputs, memory) and not on_lane(inputs), BehaviorType.next_, lambda inputs, memory: {'pre_L': inputs.center.obj})
        self._add_trans(1, 3, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs) and inputs.center.obj.turn == map.LaneTurn.LEFT_TURN, BehaviorType.TURN_LEFT)
        self._add_trans(1, 4, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs) and inputs.center.obj.turn == map.LaneTurn.RIGHT_TURN, BehaviorType.TURN_RIGHT)
        self._add_trans(1, 5, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs) and inputs.center.obj.turn == map.LaneTurn.U_TURN, BehaviorType.U_TURN)

        self._add_trans(2, 1, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs) and (inputs.center.obj in memory.pre_L.left_forward_neighbors or inputs.center.obj in memory.pre_L.right_forward_neighbors), BehaviorType.CHANGE_LANE)
        self._add_trans(2, 1, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs) and (inputs.center.obj == memory.pre_L or inputs.center.obj in memory.pre_L.successors), BehaviorType.DEPARTURE)
        self._add_trans(2, 6, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs) and (inputs.center.obj in memory.pre_L.left_reverse_neighbors or inputs.center.obj in memory.pre_L.right_reverse_neighbors), BehaviorType.OVERTAKING)
        self._add_trans(2, 2, lambda inputs, memory: not exception(inputs, memory) and not on_lane(inputs), BehaviorType.next_)

        self._add_trans(3, 3, lambda inputs, memory: not exception(inputs, memory) and inputs.center.obj.turn == map.LaneTurn.RIGHT_TURN, BehaviorType.TURN_LEFT)
        self._add_trans(3, 1, lambda inputs, memory: not exception(inputs, memory) and inputs.center.obj.turn != map.LaneTurn.RIGHT_TURN, BehaviorType.next_)

        self._add_trans(4, 4, lambda inputs, memory: not exception(inputs, memory) and inputs.center.obj.turn == map.LaneTurn.LEFT_TURN, BehaviorType.TURN_LEFT)
        self._add_trans(4, 1, lambda inputs, memory: not exception(inputs, memory) and inputs.center.obj.turn != map.LaneTurn.LEFT_TURN, BehaviorType.next_)

        self._add_trans(5, 5, lambda inputs, memory: not exception(inputs, memory) and inputs.center.obj.turn == map.LaneTurn.U_TURN, BehaviorType.U_TURN)
        self._add_trans(5, 1, lambda inputs, memory: not exception(inputs, memory) and inputs.center.obj.turn != map.LaneTurn.U_TURN, BehaviorType.next_)

        self._add_trans(6, 6, lambda inputs, memory: not exception(inputs, memory) and on_lane(inputs), BehaviorType.REVERSE_DRIVING)
        self._add_trans(6, 2, lambda inputs, memory: not exception(inputs, memory) and not on_lane(inputs), BehaviorType.next_)

        self._add_trans(7, 1, lambda inputs, memory: on_lane(inputs), BehaviorType.FOLLOW_LANE)
        self._add_trans(7, 7, lambda inputs, memory: not on_lane(inputs), BehaviorType.UNKNOWN)

    def _post_process(self, output_seq):
        if output_seq[-1] == BehaviorType.next_:
            output_seq[-1] = BehaviorType.DEPARTURE
        for i in range(len(output_seq) - 2, -1, -1):
            if output_seq[i] == BehaviorType.next_:
                output_seq[i] = output_seq[i+1]

        return output_seq

    def input(self, input_seq, reset=True):
        if reset:
            self.reset()
        output_seq = []
        for inputs in input_seq:
            output_seq.append(self._input(inputs))
        output_seq = self._post_process(output_seq)
        return output_seq

class BehaviorFactory:
    @classmethod
    def get_behavior_obj(cls, behavior_type, trace) -> BehaviorBase:
        if behavior_type == BehaviorType.FOLLOW_LANE:
            obj = FollowLaneBehavior(trace)
        elif behavior_type == BehaviorType.CHANGE_LANE:
            obj = ChangeLaneBehavior(trace)
        elif behavior_type == BehaviorType.DEPARTURE:
            obj = DepartureBehavior(trace)
        elif behavior_type == BehaviorType.OVERTAKING:
            obj = OvertakingBehavior(trace)
        elif behavior_type == BehaviorType.TURN_LEFT:
            obj = TurnLeftBehavior(trace)
        elif behavior_type == BehaviorType.TURN_RIGHT:
            obj = TurnRightBehavior(trace)
        elif behavior_type == BehaviorType.U_TURN:
            obj = UTurnBehavior(trace)
        elif behavior_type == BehaviorType.REVERSE_DRIVING:
            obj = ReverseDrivingBehavior(trace)
        elif behavior_type == BehaviorType.UNKNOWN:
            obj = UnknownBehavior(trace)
        else:
            raise ValueError(f'unsupported behavior type {behavior_type}')
        
        return obj
