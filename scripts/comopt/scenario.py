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

from itertools import filterfalse
from re import L
from loguru import logger
from comopt.simulator import common
from typing import Dict, Type, Union, overload

from lgsvl import agent
import comopt.simulator
import comopt.map as map
from comopt.utils import DictObj, DispatchMethod, check_type
from comopt.geometry import Vector, Radian
from comopt.controller import ControllerFactory
import json, copy

class AgentDescription:
    @overload
    def __init__(self, id:str, agent_type, model_name:str, init_state, controller, config=None): ...
    @overload
    def __init__(self, config): ...

    @DispatchMethod
    def __init__(self, id:str, agent_type, model_name:str, init_state, controller, config=None):
        self.id = id

        if type(agent_type) == str:
            self.agent_type:comopt.simulator.AgentType = comopt.simulator.AgentType.__members__[agent_type]
        elif type(agent_type) == comopt.simulator.AgentType:
            self.agent_type:comopt.simulator.AgentType = agent_type
        else:
            raise TypeError('Invalid agent_type', agent_type)
        
        self.model_name = model_name
        self.controller = controller
        self.config = {} if config is None else config
        self.init_state = init_state    

    @__init__.register
    def _(self, config:dict):
        self.id = config['id']
        self.agent_type = comopt.simulator.AgentType.__members__[config['agent_type']]
        self.model_name = config['model_name']
        self.init_state = comopt.simulator.AgentState(config['init_state'])
        self.controller = ControllerFactory.get_controller(config['controller'])
        self.config = config['config']

    def to_dict(self):
        ret = DictObj()
        ret['id'] = self.id
        ret['agent_type'] = self.agent_type.name
        ret['model_name'] = self.model_name
        ret['init_state'] = self.init_state.to_dict()
        ret['controller'] = self.controller.to_dict()
        ret['config'] = self.config
        return ret

class StaticItemDescription:
    @DispatchMethod
    def __init__(self, id, item_type, model_name, init_state, config=None):
        self.id = id
        if isinstance(item_type, str):
            self.item_type = comopt.simulator.StaticItemType.__members__[item_type]
        elif isinstance(item_type, comopt.simulator.AgentType):
            self.item_type = item_type
        else:
            raise TypeError('Invalid item_type', item_type)

        self.model_name = model_name
        self.init_state = init_state
        self.config = {} if config is None else config

    @__init__.register
    def _(self, config:dict):
        self.id = config['id']
        self.item_type = comopt.simulator.StaticItemType.__members__[config['item_type']]
        self.model_name = config['model_name']
        self.init_state = comopt.simulator.ObjectState(config['init_state'])
        self.config = config['config']

    def to_dict(self):
        ret = DictObj()
        ret['id'] = self.id
        ret['item_type'] = self.item_type.name
        ret['model_name'] = self.model_name
        ret['init_state'] = self.init_state.to_dict()
        ret['config'] = self.config
        return ret
        
class TrafficLightControlDescriptoin:

    @overload
    def __init__(self, traffic_light, init_color, trigger_distance, yellow_light_time=3): ...
    @overload
    def __init__(self, config): ...

    @DispatchMethod
    def __init__(self, traffic_light, init_color, trigger_distance, yellow_light_time=3):
        self.traffic_light = traffic_light
        self.init_color = init_color
        self.trigger_distance = trigger_distance
        self.yellow_light_time = yellow_light_time

    @__init__.register
    def _(self, config:dict):
        self.traffic_light = config['traffic_light']
        self.init_color = config['init_color']
        self.trigger_distance = config['trigger_distance']
        self.yellow_light_time = config['yellow_light_time']
    
    def to_dict(self):
        ret = DictObj()
        ret['traffic_light'] = self.traffic_light
        ret['init_color'] = self.init_color
        ret['trigger_distance'] = self.trigger_distance
        ret['yellow_light_time'] = self.yellow_light_time
        return ret


class ScenarioDescription:
    @overload
    def __init__(self, simulator, map_name:str, duartion=20, simulator_config=None, weather=None, time_of_day=12, agents=None, traffic_lights_control=None, record_traffic_lights=None, time_step=0.1): ...

    @overload
    def __init__(self, config:dict): ...
    @overload
    def __init__(self, config:str): ...

    @DispatchMethod
    def __init__(self, simulator, map_name:str, duration=20, simulator_config=None, weather=None, time_of_day=12, agents=None, static_items=None, traffic_lights_control=None, record_traffic_lights=None, time_step=0.1, aut_id='aut'):
        
        if type(simulator) == str:
            self.simulator:comopt.simulator.SimulatorType = comopt.simulator.SimulatorType.__members__[simulator]
        elif type(simulator) == comopt.simulator.SimulatorType:
            self.simulator:comopt.simulator.SimulatorType = simulator
        else:
            raise AssertionError('Invalid simulator', simulator)

        self.configuraiton = self._get_simulator_configuration()

        self.simulator_config = self.configuraiton.default_simulator_config
        if not simulator_config is None:
            self.simulator_config.update(simulator_config)
        
        self._simulator_obj = None

        self.map_name = map_name
        self._map = None
        self.weather = self.configuraiton.default_environment_config('weather')
        if not weather is None:
            self.weather.update(weather)
        self.time_of_day = time_of_day
        
        self.agents = {} if agents is None else agents
        self.static_items = {} if static_items is None else static_items
        self.traffic_lights_control = [] if traffic_lights_control is None else traffic_lights_control
        self.record_traffic_lights = [] if record_traffic_lights is None else record_traffic_lights
        self.duration = duration
        self.time_step = time_step
        self.aut_id = aut_id

    @__init__.register
    def __init_by_dcit(self, config:dict):
        self._parse_from_config(config)

    @__init__.register
    def __init_by_file(self, config:str):
        with open(config, 'r') as f:
            self._parse_from_config(json.load(f))

    def _parse_from_config(self, config:dict):
        if 'scenario' in config:
            config = config['scenario']
        self.simulator = comopt.simulator.SimulatorType.__members__[config['simulator']]
        self.simulator_config = config['simulator_config']
        self._simulator_obj = None
        self.map_name = config['map_name']
        self._map = None
        self.weather = DictObj(config['weather'])
        self.time_of_day = config['time_of_day']
        self.agents = {}
        for agent in config['agents']:
            self.agents[agent['id']] = AgentDescription(agent)
        self.static_items = {}
        for static_item in config['static_items']:
            self.static_items[static_item['id']] = StaticItemDescription(static_item)
        self.traffic_lights_control = []
        for traffic_lights_control_config in config['traffic_lights_control']:
            self.traffic_lights_control.append(TrafficLightControlDescriptoin(traffic_lights_control_config))
        self.record_traffic_lights = []
        for traffic_lights_id in config['record_traffic_lights']:
            self.record_traffic_lights.append(traffic_lights_id)
        self.duration = config['duration']
        self.time_step = config['time_step']
        self.aut_id = config['aut_id']

    def _get_simulator(self):
        if self.simulator == comopt.simulator.SimulatorType.SVL:
            return comopt.simulator.SVLSimulator(**self.simulator_config)
        else:
            raise NotImplementedError

    def _get_simulator_configuration(self):
        if self.simulator == comopt.simulator.SimulatorType.SVL:
            return comopt.simulator.svl_configuration
        else:
            raise NotImplementedError

    def _drop_simulator(self):
        if not self._simulator_obj is None:
            self.simulator_obj.close()
        self._simulator_obj = None

    def to_dict(self):
        ret = DictObj()
        ret['simulator'] = self.simulator.name
        ret['simulator_config'] = self.simulator_config
        ret['map_name'] = self.map_name
        weather_dict = self.weather
        self._drop_simulator()
        weather_dict.update(self.weather)
        ret['weather'] = weather_dict
        ret['time_of_day'] = self.time_of_day
        ret['agents'] = []
        for id, agent in self.agents.items():
            ret['agents'].append(agent.to_dict())
        ret['static_items'] = []
        for id, static_item in self.static_items.items():
            ret['static_items'].append(static_item.to_dict())
        ret['traffic_lights_control'] = []
        for traffic_ligth_control in self.traffic_lights_control:
            ret['traffic_lights_control'].append(traffic_ligth_control.to_dict())
        ret['record_traffic_lights'] = []
        for traffic_light in self.record_traffic_lights:
            ret['record_traffic_lights'].append(traffic_light)
        ret['duration'] = self.duration
        ret['time_step'] = self.time_step
        ret['aut_id'] = self.aut_id
        return ret

    def save(self, file_path):
        data = self.to_dict()
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=2)

    @property
    def available(self):
        agent_list = [agent for agent in self.agents.values()]
        for a in agent_list:
            if a.init_state is None:
                return False
        for a1 in agent_list:
            for a2 in agent_list:
                if not a1 is a2:
                    if (
                        a1.agent_type == comopt.simulator.common.AgentType.PEDESTRIAN and \
                        a2.agent_type == comopt.simulator.common.AgentType.PEDESTRIAN 
                    ):
                        continue
                    try:
                        a1_lane_info = self.map.get_closest_lane(a1.init_state.position, check_bound=False)
                        a2_lane_info = self.map.get_closest_lane(a2.init_state.position, check_bound=False)
                        if not a1_lane_info.obj == a2_lane_info.obj:
                            continue
                        a1_init_speed = a1.init_state.velocity.magnitude()
                        # print(a2_lane_info.s, a1_lane_info.s, a2_lane_info.s + (a1_init_speed ** 2 / 2 / 6.94) + 2)
                        if a1_lane_info.s <= a2_lane_info.s < a1_lane_info.s + (a1_init_speed ** 2 / 2 / 4) + 2:
                            return False
                    except Exception as e:
                        return False
                    # if  a1.init_state.dis_to(a2.init_state) < 2:
                    #     return False
        return True

    def exec(self):
        assert self.available, 'The scenario is unavailable'
        self._drop_simulator()
        if self.time_step is None:
            self.simulator_obj.exec(self, self.duration)
            self._drop_simulator()
        else:
            trace = None
            while trace is None:
                try:
                    self._drop_simulator()
                    trace = self.simulator_obj.exec(self, self.duration, self.time_step)
                except Exception as e:
                    logger.warning(f"Exception occured when the simulator execute sceanrio: {e}. Retrying.")
            self._drop_simulator()
            return trace

    # @check_type
    def add_agent(self, agent_description:AgentDescription):
        assert not agent_description.id in self.agents and not agent_description.id in self.agents
        self.agents[agent_description.id] = agent_description

    def add_static_item(self, static_item_description:StaticItemDescription):
        assert not static_item_description.id in self.static_items and not static_item_description.id in self.agents
        self.static_items[static_item_description.id] = static_item_description

    def add_traffic_light_control(self, traffic_light_control_description):
        self.traffic_lights_control.append(
            traffic_light_control_description
        )
    
    def add_record_traffic_light(self, traffic_light):
        if isinstance(traffic_light, str):
            self.record_traffic_lights.append(traffic_light)
        else:
            self.record_traffic_lights.append(traffic_light.id)

    @property
    def map(self):
        if not self._map is None:
            return self._map
        self._map = map.Map(self.simulator, self.map_name)
        return self._map

    @property
    def simulator_obj(self):
        if self._simulator_obj is None:
            self._simulator_obj = self._get_simulator()
        return self._simulator_obj

    def copy(self):
        self._drop_simulator()
        # ret = ScenarioDescription(self.simulator, self.map_name, self.duration)
        # ret.simulator_config = self.simulator_config.copy()

        # # environment information
        # ret.weather = self.weather.copy()
        # ret.time_of_day = self.time_of_day
        
        # # agents {agent_id : agent_description}
        # ret.agents = copy.deepcopy(self.agents)
        # ret.static_items = copy.deepcopy(self.static_items)
        # ret.traffic_lights_control = copy.deepcopy(self.traffic_lights_control.copy)
        # ret.time_step = self.time_step

        # return ret

        return ScenarioDescription(self.to_dict())
