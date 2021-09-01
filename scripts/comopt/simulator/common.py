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

from loguru import logger
import comopt.map as map
import comopt.scenario as scenario
from comopt.analyzer.behavior import BehaviorType
import comopt.analyzer as analyzer
from comopt.geometry import Vector, Segment, Radian
from comopt.utils import DictObj, DispatchMethod, TimeFunction, check_type
from comopt.controller import ControllerBase
from typing import Union, overload
import math
from functools import cached_property
from enum import Enum
import json, time

class CollisionException(BaseException):
    def __str__(self) -> str:
        return 'Collision occured'

class SimulatorType(Enum):
    SVL = 1
    LGSVL = 2

class AgentType(Enum):
    EGO = 1
    NPC = 2
    PEDESTRIAN = 3


class StaticItemType(Enum):
    Cone = 1

class TrafficLightState(Enum):
    GREEN = 1
    RED = 2
    YELLOW = 3


class AgentState:
    @DispatchMethod
    def __init__(self, position:Vector=Vector(0,0), heading:Radian=Radian(0), velocity:Vector=Vector(0,0), angular_velocity:Union[int, float]=0, width:Union[int, float] = 0, length:Union[int, float] = 0):
        self.position = position
        self.heading = heading
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.width = width
        self.length = length
        self._gen_frame_segments()

    @__init__.register
    def _(self, config:dict):
        self.position = Vector(config['position'])
        self.heading = Radian(config['heading'])
        self.velocity = Vector(config['velocity'])
        self.angular_velocity = config['angular_velocity']
        self.width = config['width']
        self.length = config['length']
        self._gen_frame_segments()

    def _gen_frame_segments(self):
        self._segments = None
        self._points = None
        

    def to_dict(self):
        ret = dict()
        ret['position'] = self.position.to_dict()
        ret['heading'] = self.heading.to_dict()
        ret['velocity'] = self.velocity.to_dict()
        ret['angular_velocity'] = self.angular_velocity
        ret['width'] = self.width
        ret['length'] = self.length
        return ret
    
    @cached_property
    def points(self):
        self._points = tuple(self.corner_position(loc) for loc in ('fr', 'br', 'bl', 'fl'))
        return self._points

    @property
    def segments(self):
        self._segments = tuple(Segment(self.points[i], self.points[(i+1)%4]) for i in range(4))
        return self._segments

    def dis_to(self, other) -> float:

        min_dis = float('inf')

        for point in self.points:
            for segment in other.segments:
                min_dis = min(min_dis, segment.dis_to_point(point).distance)

        for point in other.points:
            for segment in self.segments:
                min_dis = min(min_dis, segment.dis_to_point(point).distance)

        return min_dis


    def corner_position(self, corner) -> Vector:
        '''
        corner in ['fr', 'br', 'fl', 'bl'], 'f': front; 'b': back; 'r': right; 'l': left.
        '''
        assert len(corner) == 2
        assert ('r' in corner) ^ ('l' in corner)
        assert ('f' in corner) ^ ('b' in corner)
        
        x = 0
        if 'r' in corner:
            x = self.width / 2
        elif 'l' in corner:
            x = -self.width / 2


        y = 0
        if 'f' in corner:
            y = self.length / 2
        elif 'b' in corner:
            y = -self.length / 2

        return Vector(
                math.cos((self.heading-Radian(math.pi/2)).r) * x - math.sin((self.heading-Radian(math.pi/2)).r) * y,
                math.sin((self.heading-Radian(math.pi/2)).r) * x + math.cos((self.heading-Radian(math.pi/2)).r) * y
            ) + self.position
    
    # @check_type
    def right(self, l:Union[int, float]):
        return AgentState(
            position=self.position + (self.heading - math.pi / 2).unit * l,
            heading=self.heading,
            velocity=self.velocity.copy(),
            angular_velocity=self.angular_velocity
        )
    
    # @check_type
    def forward(self, l:Union[int, float]):
        return AgentState(
            position=self.position + self.heading.unit * l,
            heading=self.heading,
            velocity=self.velocity.copy(),
            angular_velocity=self.angular_velocity
        )
    
    # @check_type
    def rotate(self, r:Union[int, float, Radian]):
        return AgentState(
            position=self.position.copy(),
            heading=self.heading + r,
            velocity=self.velocity.copy(),
            angular_velocity=self.angular_velocity
        )
    
    def abs_to_rel(self, state):
        is_vector = False
        if isinstance(state, Vector):
            is_vector = True
            state = AgentState(position=state)
        abs_position = state.position

        abs_position = state.position - self.position
        rel_position = Vector(
            math.cos((self.heading-Radian(math.pi/2)).r) * abs_position.x + math.sin((self.heading-Radian(math.pi/2)).r) * abs_position.y,
            -math.sin((self.heading-Radian(math.pi/2)).r) * abs_position.x + math.cos((self.heading-Radian(math.pi/2)).r) * abs_position.y
        )

        rel_heading = Radian(state.heading.r - self.heading.r)

        abs_velocity = state.velocity
        abs_velocity = state.velocity - self.velocity
        rel_velocity = Vector(
            math.cos((self.heading-Radian(math.pi/2)).r) * abs_velocity.x + math.sin((self.heading-Radian(math.pi/2)).r) * abs_velocity.y,
            -math.sin((self.heading-Radian(math.pi/2)).r) * abs_velocity.x + math.cos((self.heading-Radian(math.pi/2)).r) * abs_velocity.y
        )
        rel_angular_velocity = state.angular_velocity - self.angular_velocity
        rel_state = AgentState(
            position=rel_position,
            heading=rel_heading,
            velocity=rel_velocity,
            angular_velocity=rel_angular_velocity
        )
        dict
        if is_vector:
            return rel_state.position
        else:
            return rel_state

    @overload
    def rel_to_abs(self, state:Vector) -> Vector: ...
    @overload
    def rel_to_abs(self, state:'AgentState') -> 'AgentState': ...

    def rel_to_abs(self, state):
        is_vector=False
        _state = state
        if isinstance(state, Vector):
            rel_position = state
            return Vector(
                math.cos((self.heading-Radian(math.pi/2)).r) * rel_position.x - math.sin((self.heading-Radian(math.pi/2)).r) * rel_position.y,
                math.sin((self.heading-Radian(math.pi/2)).r) * rel_position.x + math.cos((self.heading-Radian(math.pi/2)).r) * rel_position.y
            ) + self.position
        rel_position = _state.position
        abs_position = Vector(
            math.cos((self.heading-Radian(math.pi/2)).r) * rel_position.x - math.sin((self.heading-Radian(math.pi/2)).r) * rel_position.y,
            math.sin((self.heading-Radian(math.pi/2)).r) * rel_position.x + math.cos((self.heading-Radian(math.pi/2)).r) * rel_position.y
        ) + self.position
        abs_heading = Radian(_state.heading.r + self.heading.r)
        rel_velocity = _state.velocity
        abs_velocity = Vector(
            math.cos((self.heading-Radian(math.pi/2)).r) * rel_velocity.x - math.sin((self.heading-Radian(math.pi/2)).r) * rel_velocity.y,
            math.sin((self.heading-Radian(math.pi/2)).r) * rel_velocity.x + math.cos((self.heading-Radian(math.pi/2)).r) * rel_velocity.y
        ) + self.velocity
        abs_angular_velocity = _state.angular_velocity + self.angular_velocity
        abs_state = AgentState(
            position=abs_position,
            heading=abs_heading,
            velocity=abs_velocity,
            angular_velocity=abs_angular_velocity
        )
        if is_vector:
            return abs_state.position
        else:
            return abs_state

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return f'{{ position: {self.position}, heading: {self.heading}, velocity: {self.velocity}, angular_velocity: {self.angular_velocity} rad/s }}'

class SimulationResult:
    @DispatchMethod
    def __init__(self, scenario_description):
        self.scenario = scenario_description
        self.traces = dict()
        self.traffic_lights = dict()
        self.aut_id = scenario_description.aut_id
        self.collision = False
        self._aut_analyzer = None

    @__init__.register
    def _(self, config:dict):
        self.scenario = scenario.ScenarioDescription(config['scenario'])
        self.aut_id = self.scenario.aut_id
        self.traces = dict()
        for id, val in config['traces'].items():
            self.traces[id] = TimeFunction()
            for t, state_conf in val.items():
                self.traces[id][float(t)] = AgentState(state_conf)
        self.traffic_lights = dict()
        for id, val in config['traffic_lights'].items():
            self.traffic_lights[id] = TimeFunction()
            for t, light_state in val.items():
                self.traffic_lights[id][float(t)] = TrafficLightState.__members__[light_state]
        self.collision = config['collision']
        self._aut_analyzer = None

    def to_dict(self):
        ret = dict()
        ret['scenario'] = self.scenario.to_dict()
        ret['traces'] = dict()
        for id, trace in self.traces.items():
            ret['traces'][id] = trace.to_dict()
        ret['traffic_lights'] = dict()
        for id, light_seq in self.traffic_lights.items():
            ret['traffic_lights'][id] = light_seq.to_dict()
        ret['collision'] = self.collision
        # ret['report'] = self.report
        return ret

    def save(self, file_path, report=False):
        data = self.to_dict()
        if report:
            data['report'] = self.report
        with open(file_path, 'w') as f:
            json.dump(data, f)

    def __setitem__(self, k: str, v: TimeFunction) -> None:
        self.traces[k] = v
    
    def __getitem__(self, key: str):
        return self.traces[key]

    @property
    def ticks(self):
        return self.traces[self.aut_id].ticks

    @property
    def time_step(self):
        ticks = self.ticks
        return ticks[1] - ticks[0]

    @cached_property
    def aut_min_dis_to_others(self):
        min_dis = float('inf')
        for t in self.ticks:
            aut_state = self.traces[self.aut_id][t]
            for key, trace in self.traces.items():
                if key != self.aut_id:
                    agent_state = trace[t]
                    min_dis = min(min_dis, aut_state.dis_to(agent_state))
        return min_dis

    @cached_property
    def aut_max_acc(self):
        max_acc = 0
        pre_t = None
        pre_v = None
        for t, state in self.traces[self.aut_id].iter():
            if not pre_t is None and not pre_v is None:
                if state.velocity.magnitude() > pre_v.magnitude():
                    max_acc = max(max_acc, (state.velocity - pre_v).magnitude() / (t - pre_t))
            pre_v = state.velocity
            pre_t = t
        return max_acc

    @cached_property
    def aut_ignore_lights(self):
        pre_t = None
        pre_state = None
        RunLightRecord = namedtuple('RunLightRecord', ['time', 'traffic_light_id', 'color'])
        ignore_lights_record = []
        for t, aut_state in self.traces[self.aut_id].iter():
            if not pre_t is None and not pre_state is None:
                for traffic_light_id, light_state in self.traffic_lights.items():
                    traffic_light = self.scenario.map.traffic_lights[traffic_light_id]
                    for segment in traffic_light.stop_segments:
                        segment:Segment = segment
                        if segment.cover_parallelly(pre_state.position) and segment.cover_parallelly(aut_state.position) and \
                            (segment.to_left_test(pre_state.position) ^ segment.to_left_test(aut_state.position)) and \
                            (light_state[t] != TrafficLightState.GREEN):
                            ignore_lights_record.append(RunLightRecord(t, traffic_light_id, light_state[t].name))
            pre_t = t
            pre_state = aut_state
                
        return tuple(ignore_lights_record)


    @cached_property
    def aut_speed(self):
        ret = TimeFunction()
        for t, state in self.traces[self.aut_id].iter():
            ret[t] = state.velocity.magnitude()
        return ret

    @cached_property
    def aut_lateral_jerk(self):
        try:
            lane_info = self.aut_analyzer.lane_info
        except map.OutOfRoadError:
            lane_info = self.aut_analyzer.lane_info
        ticks = lane_info.ticks
        max_jerk = 0
        ret = TimeFunction()
        for i in range(0, len(ticks) - 3):
            if lane_info[ticks[i]].center.obj == \
               lane_info[ticks[i+1]].center.obj == \
               lane_info[ticks[i+2]].center.obj == \
               lane_info[ticks[i+3]].center.obj:
                jerk = - lane_info[ticks[i]].center.l \
                       + 3 * lane_info[ticks[i+1]].center.l \
                       - 3 * lane_info[ticks[i+2]].center.l \
                       + lane_info[ticks[i+3]].center.l
                ret[ticks[i+1]] = jerk
        return ret

    @cached_property
    def aut_acc(self):
        ret = TimeFunction()
        pre_t = None
        pre_v = None
        for t, state in self.traces[self.aut_id].iter():
            if not pre_t is None and not pre_v is None:
                ret[t] = (state.velocity - pre_v).magnitude() / (t - pre_t) * (-1 if state.velocity.magnitude() < pre_v.magnitude() else 1)
            pre_v = state.velocity
            pre_t = t
        return ret

    @cached_property
    def aut_max_brake(self):
        max_brake = 0
        pre_t = None
        pre_v = None
        for t, state in self.traces[self.aut_id].iter():
            if not pre_t is None and not pre_v is None:
                if state.velocity.magnitude() < pre_v.magnitude():
                    max_brake = max(max_brake, (state.velocity - pre_v).magnitude() / (t - pre_t))
            pre_v = state.velocity
            pre_t = t
        return max_brake

    @cached_property
    def aut_dis_to_dest(self):
        ticks = self.ticks
        aut_agent = self.scenario.agents[self.aut_id]
        final_distance = self.traces[self.aut_id][ticks[-1]].position.dis_to(aut_agent.controller.destination)
        return final_distance

    @cached_property
    def has_unknown_behavior(self):
        for behavior in self.aut_analyzer.behaviors:
            if behavior.type == BehaviorType.UNKNOWN:
                return True
        return False

    @cached_property
    def aut_max_lateral_jerk(self):
        try:
            lane_info = self.aut_analyzer.lane_info
        except map.OutOfRoadError:
            lane_info = self.aut_analyzer.lane_info
        ticks = lane_info.ticks
        max_jerk = 0
        time_step = self.time_step
        for i in range(0, len(ticks) - 3):
            if lane_info[ticks[i]].center.obj == \
               lane_info[ticks[i+1]].center.obj == \
               lane_info[ticks[i+2]].center.obj == \
               lane_info[ticks[i+3]].center.obj:
                jerk = - lane_info[ticks[i]].center.l \
                       + 3 * lane_info[ticks[i+1]].center.l \
                       - 3 * lane_info[ticks[i+2]].center.l \
                       + lane_info[ticks[i+3]].center.l
                max_jerk = max(max_jerk, abs(jerk))
        return max_jerk

    @property
    def aut_analyzer(self):
        if not self._aut_analyzer is None:
            return self._aut_analyzer
        self._aut_analyzer = analyzer.TraceAnalyzer(
            self.traces[self.aut_id], 
            self.scenario.map
        )
        return self._aut_analyzer

    @cached_property
    def report(self):
        ret = dict()
        ret['exception'] = []
        ret['minimum_distance'] = 'inf' if self.aut_min_dis_to_others == float('inf') else self.aut_min_dis_to_others
        ret['ditance_to_destination'] = self.aut_dis_to_dest
        ret['maximum_acceleration'] = self.aut_max_acc
        ret['maximum_brake'] = self.aut_max_brake
        ret['maximum_lateral_jerk'] = self.aut_max_lateral_jerk
        ret['ignore_lights'] = self.aut_ignore_lights
        
        if self.aut_min_dis_to_others < 0.5:
            ret['exception'].append('TOO_CLOSE')
        if self.collision == True:
            ret['exception'].append('COLLISION')
        if self.aut_analyzer.out_of_road:
            ret['exception'].append('OUT_OF_ROAD')
        if len([record for record in self.aut_ignore_lights if record.color == TrafficLightState.YELLOW.name]) > 0:
            ret['exception'].append('IGNORE_YELLOW_LIGHT')
        if len([record for record in self.aut_ignore_lights if record.color == TrafficLightState.RED.name]) > 0:
            ret['exception'].append('IGNORE_RED_LIGHT')
        if self.aut_dis_to_dest > 10:
            ret['exception'].append('UNREACHED')
        if self.aut_max_lateral_jerk > 0.4:
            ret['exception'].append('DISCOMFORT_LATERAL_JERK')
        if self.aut_max_acc > 6.94:
            ret['exception'].append('DISCOMFORT_ACCELERATION')
        if self.aut_max_brake > 6.94:
            ret['exception'].append('DISCOMFORT_BRAKE')
        if BehaviorType.UNKNOWN in [behavior.type for behavior in self.aut_analyzer.behaviors]:
            ret['exception'].append('UNKNOWN_BEHAVIOR')
        return ret

    def replay(self, time_step=0):

        simulator = self.scenario.simulator_obj
        scenario_description = self.scenario
        simulator.load_map(scenario_description.map_name)
        simulator.set_weather(scenario_description.weather)
        simulator.set_time_of_day(scenario_description.time_of_day)

        for id, agent_description in scenario_description.agents.items():
            agent = simulator.add_agent(id, agent_description.agent_type, agent_description.model_name, agent_description.config)
            agent.state = agent_description.init_state
            # agent.apply_controller(agent_description.controller)

        try:
            for t in self.ticks:
                for agent_id, trace in self.traces.items():
                    agent = simulator.get_agent_by_id(agent_id)
                    agent.state = trace[t]
                simulator.run(0.01)
                if time_step > 0:
                    time.sleep(time_step)
        except KeyboardInterrupt:
            logger.warning(f"Scenario replaying cancel by user.")
        except CollisionException:
            logger.warning(f"Collision occured")
        simulator.close()
        

class ObjectState(AgentState):
    pass

class AgentBase:
    def __init__(self, simulator, id:str, agent_type:AgentType, model_name:str, config=None):
        self.id = id
        self.type = agent_type
        self.model_name = model_name
        self.config = {} if config is None else config
        self.simulator = simulator
        self.controller = None
        self.sim_obj = None
        self.width=0
        self.length=0
        if agent_type == AgentType.NPC:
            self._init_npc()
        if agent_type == AgentType.EGO:
            self._init_ego()
        if agent_type == AgentType.PEDESTRIAN:
            self._init_pedestrian()

        self.state = AgentState()
    
    def apply_controller(self, controller:ControllerBase):
        self.controller = controller
        method_name = '_apply_' + controller.name
        if method_name in dir(self):
            getattr(self, method_name)()
        else:   
            raise AssertionError(f"{controller.name} is not Supported by SVL Simulator.")

    def update_control(self):
        if not self.controller is None:
            method_name = '_update_' + self.controller.name
            if method_name in dir(self):
                getattr(self, method_name)()
    
    def _init_npc(self):
        raise NotImplementedError

    def _init_ego(self):
        raise NotImplementedError
    
    def _init_pedestrian(self):
        raise NotImplementedError



class StaticItemBase:
    def __init__(self, simulator, id, item_type, model_name, config=None):
        self.simulator = simulator
        self.id = id
        self.type = item_type
        self.model_name = model_name
        self.config = {} if config is None else config
        self.sim_obj = None
        self.width = 0
        self.length = 0
        getattr(self, '_init_' + item_type.name.lower())()
        self.state = ObjectState()

class SimulatorBase:
    def __init__(self):
        self._agents = dict()
        self._static_items = dict()

    def add_agent(self, id:str, agent_type:AgentType, model_name:str, config:dict=dict()):
        raise NotImplementedError
    
    def add_static_item(self, id, item_type, model_name, config):
        raise NotImplementedError

    def reset(self):
        self._agents = dict()

    def get_agent_by_id(self, id:str):
        assert id in self._agents, f'Agent {id} was not found.'
        return self._agents[id]

    def run(self, time_limit):
        raise NotImplementedError

    def load_map(self, map_name):
        raise NotImplementedError

    def set_weather(self, weather):
        raise NotImplementedError

    def set_time_of_day(self, t):
        raise NotImplementedError

    @property
    def agents(self):
        return self._agents
    
    @property
    def static_items(self):
        return self._static_items

    def close(self):
        for id, agent in self.agents.items():
            if not agent.controller is None:
                agent.controller.close()

    @property
    def signals(self):
        result = dict()
        for id, agent in self.agents.items():
            result[id] = agent.state
        for id, item in self.static_items.items():
            result[id] = item.state
        return result

    def set_traffic_light(self, traffic_ligth, init_color, trigger_distance, yellow_time=2):
        raise NotImplementedError

    def get_traffic_light_state(self, traffic_light):
        return NotImplementedError

    @overload
    def exec(self, scenario_description, time_limit) -> None: ...

    @overload
    def exec(self, scenario_description, time_limit, time_step) -> SimulationResult: ...

    def exec(self, scenario_description, time_limit, time_step=None):
        # self.reset()
        self.load_map(scenario_description.map_name)
        self.set_weather(scenario_description.weather)
        self.set_time_of_day(scenario_description.time_of_day)

        for id, agent_description in scenario_description.agents.items():
            agent = self.add_agent(id, agent_description.agent_type, agent_description.model_name, agent_description.config)
            agent.state = agent_description.init_state
            agent.apply_controller(agent_description.controller)
        
        for id, static_item_description in scenario_description.static_items.items():
            static_item = self.add_static_item(id, static_item_description.item_type, static_item_description.model_name, static_item_description.config)
            static_item.state = static_item_description.init_state

        for traffic_light_control in scenario_description.traffic_lights_control:
            self.set_traffic_light(
                scenario_description.map.traffic_lights[traffic_light_control.traffic_light],
                traffic_light_control.init_color,
                traffic_light_control.trigger_distance,
                traffic_light_control.yellow_light_time
            )

        result = None   
        
        if time_step is None:
            # self.run(0.1)
            # time.sleep(1)
            try:
                self.run(time_limit)
            except CollisionException:
                return
        else:
            def reach_destination():
                aut = self.agents[scenario_description.aut_id]
                destination = aut.controller.destination
                if aut.state.velocity.magnitude() < 0.01 and aut.state.position.dis_to(destination) < 10:
                    return True
                return False

            result = SimulationResult(scenario_description)
            for k in self.agents.keys():
                result[k] = TimeFunction()
            for id in self.static_items.keys():
                result[id] = TimeFunction()
            for id in scenario_description.record_traffic_lights:
                result.traffic_lights[id] = TimeFunction()
            cur_time = 0
            
            while cur_time < time_limit:
                for id, state in self.signals.items():
                    result[id].add(cur_time, state)
                for agent in self.agents.values():
                    agent.update_control()
                
                for traffic_light_id in scenario_description.record_traffic_lights:
                    result.traffic_lights[traffic_light_id].add(cur_time, self.get_traffic_light_state(scenario_description.map.traffic_lights[traffic_light_id]))

                cur_time += time_step
                
                if reach_destination():
                    return result

                try:
                    self.run(time_step)
                except CollisionException:
                    result.collision = True
                    for id, state in self.signals.items():
                        result[id].add(cur_time, state)
                    return result
                
        for agent in self.agents.values():
            agent.controller.close()

        return result

