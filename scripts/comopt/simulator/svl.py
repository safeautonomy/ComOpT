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

# from comopt.map import transform
from collections import namedtuple
from pickle import REDUCE
import lgsvl, os, copy

from comopt.simulator.common import SimulatorBase, AgentType, AgentState, AgentBase, SimulatorType, CollisionException, StaticItemBase, StaticItemType, TrafficLightState
from enum import Enum
from comopt.geometry import Vector, Radian
from comopt.utils import check_type
from comopt.controller import ApolloController, LaneFollowController, ControllerBase
import comopt.map
import json, math, logging, time
from loguru import logger
from comopt.config import LIBDIR

INF = 10 ** 10
MAX_SIGNAL_DISTANCE = 1


class SVLConfiguration:
    def __init__(self, model_config_file, environment_config_file, simulator_config_file):
        with open(model_config_file) as f:
            model_config = json.load(f)
        with open(environment_config_file) as f:
            self.environment_config = json.load(f)
        with open(simulator_config_file) as f:
            self.simulator_config = json.load(f)['SVL']
        self.model_config = dict()
        self.model_config[AgentType.EGO] = model_config['EgoModels']
        self.model_config[AgentType.NPC] = model_config['NPCModels']
        self.model_config[AgentType.PEDESTRIAN] = model_config['PedestrianModels']
        self.model_config[StaticItemType.Cone] = model_config['ConeModels']

    @check_type
    def models(self, model_type):
        if isinstance(model_type, str):
            model_type = AgentType.__members__[model_type]
        models = []
        for k, v in self.model_config[model_type].items():
            if 'svl_config' in v:
                models.append(k)
        return models

    @check_type
    def default_environment_config(self, param:str):
        assert 'svl_config' in self.environment_config[param]
        return self.environment_config[param]['svl_config']['default']

    @property
    def default_simulator_config(self):
        return self.simulator_config['default']

    @check_type
    def model_info(self, model_type, model_name: str):
        assert model_name in self.models(model_type), f'`{model_name}` is not available in SVL Simulator.'
        return self.model_config[model_type][model_name]['svl_config']

    def ego_id(self, model_name: str, mode):
        assert model_name in self.models(AgentType.EGO)
        if mode is None:
            return self.model_config[AgentType.EGO][model_name]['svl_config']['default_config']['id']
        else:
            for config in self.model_config[AgentType.EGO][model_name]['svl_config']['configs']:
                if config['mode'] == mode:
                    return config['id']
            raise AssertionError(f'Can not find the configuration {mode}')

    def get_size(self, agent_type: AgentType, model_name: str):
        model_info = self.model_info(agent_type, model_name)
        assert 'size' in model_info
        AgentSize = namedtuple('AgentSize', ['length', 'width'])
        return AgentSize(model_info['size']['length'], model_info['size']['width'])

    @check_type
    def sim_type(self, agent_type: AgentType):
        if agent_type == AgentType.EGO:
            return lgsvl.AgentType.EGO
        elif agent_type == AgentType.NPC:
            return lgsvl.AgentType.NPC
        elif agent_type == AgentType.PEDESTRIAN:
            return lgsvl.AgentType.PEDESTRIAN

    # @check_type
    def std_from_sim(self, sim_vec: lgsvl.Vector):
        return Vector(sim_vec.z, -sim_vec.x, sim_vec.z)

    # @check_type
    def std_to_sim(self, std_vec: Vector):
        return lgsvl.Vector(-std_vec.y, std_vec.z, std_vec.x)

    def gen_traffic_light_policy(self, effective, color, trigger_distance, yellow_light_time):
        if not effective:
            return {
                'self': 'yellow=120;loop',
                'reverse': 'yellow=120;loop'
            }
        red_policy = f'red=1;trigger={trigger_distance};green=120'
        green_policy = f'green=1;trigger={trigger_distance};yellow={yellow_light_time};red=22;green=120'
        if color == 'red':
            return {
                'self': red_policy,
                'reverse': green_policy
            }
        elif color == 'green':
            return {
                'self': green_policy,
                'reverse': red_policy
            }

    def std_state_to_sim(self, sim_obj, std_state: AgentState):
        if isinstance(sim_obj, SVLAgent):
            sim_state = sim_obj.sim_obj.state
        elif isinstance(sim_obj, SVLStaticItem):
            sim_state = sim_obj.sim_obj.object_state
        else:
            raise TypeError
        sim_state.rotation.y = -std_state.heading.angle
        sim_position = self.std_to_sim(std_state.position)
        sim_state.transform.position.x, sim_state.transform.position.y, sim_state.transform.position.z = sim_position.x, sim_position.y, sim_position.z
        sim_state.transform.position.y += 1000
        hit = sim_obj.simulator.svl_sim.raycast(sim_state.position, lgsvl.Vector(0, -1, 0), 1)
        assert not hit is None, 'Raycast returns None.'
        sim_state.transform.position.x, sim_state.transform.position.y, sim_state.transform.position.z = hit.point.x, hit.point.y, hit.point.z
        sim_state.velocity = self.std_to_sim(std_state.velocity)
        sim_state.angular_velocity = lgsvl.Vector(0, -std_state.angular_velocity / math.pi * 180, 0)
        return sim_state


svl_configuration = SVLConfiguration(
    os.path.join(LIBDIR, './config/models.json'),
    os.path.join(LIBDIR, './config/environment.json'),
    os.path.join(LIBDIR, './config/simulator.json')
)


class SVLAgent(AgentBase):

    def _on_collision(self, agent1=None, agent2=None, contact=None):
        # print('collision', type(agent1), type(agent2), contact)
        if type(agent2) == lgsvl.agent.Pedestrian and agent1.state.velocity.magnitude() < 0.2:
            return
        logger.warning(f'Collision occured')
        raise CollisionException

    def _init_pedestrian(self):
        self.sim_obj = self.simulator.svl_sim.add_agent(
            svl_configuration.model_info(self.type, self.model_name)['model_name'], 
            svl_configuration.sim_type(self.type)
        )

    def _init_ego(self):
        mode = None
        if 'mode' in self.config:
            mode = self.config['mode']
        self.sim_obj = self.simulator.svl_sim.add_agent(
            svl_configuration.ego_id(self.model_name, mode),
            svl_configuration.sim_type(self.type)
        )
        self.sim_obj.on_collision(self._on_collision)
        self.length, self.width = svl_configuration.get_size(self.type, self.model_name)

    def _init_npc(self):
        color = lgsvl.Vector(-1, -1, -1)
        if 'color' in self.config:
            assert 0 <= self.config['color']['R'] <= 1 and \
                   0 <= self.config['color']['G'] <= 1 and \
                   0 <= self.config['color']['B'] <= 1, \
                '`RGB` of color should in 0..1 range.'

            color = lgsvl.Vector(self.config['color']['R'], self.config['color']['G'], self.config['color']['B'])

        self.sim_obj = self.simulator.svl_sim.add_agent(
            svl_configuration.model_info(self.type, self.model_name)['model_name'],
            svl_configuration.sim_type(self.type),
            color=color
        )
        self.length, self.width = svl_configuration.get_size(self.type, self.model_name)
        

    @property
    def state(self):
        sim_state = self.sim_obj.state
        return AgentState(
            position=svl_configuration.std_from_sim(sim_state.position),
            heading=Radian(-sim_state.rotation.y / 180 * math.pi),
            velocity=svl_configuration.std_from_sim(sim_state.velocity),
            angular_velocity=-math.radians(sim_state.angular_velocity.y),
            width=self.width,
            length=self.length
        )

    

    @state.setter
    def state(self, std_state: AgentState):
        sim_state = svl_configuration.std_state_to_sim(self, std_state)
        self.sim_obj.state = sim_state

    def _apply_static_controller(self):
        pass

    def _apply_single_destination_controller(self):
        trigger_distance = self.controller.trigger_distance
        speed = self.controller.speed
        destination = self.controller.destination
        if self.type == AgentType.PEDESTRIAN:
            waypoints = [
                lgsvl.WalkWaypoint(self.sim_obj.transform.position, speed=speed, idle=0, trigger_distance=trigger_distance),
                lgsvl.WalkWaypoint(svl_configuration.std_state_to_sim(self, AgentState(destination)).position, 0, speed=speed)
            ]
            self.sim_obj.follow(waypoints, loop=False)
        else:
            raise NotImplementedError

    def _apply_lane_follow_controller(self):
        assert self.type == AgentType.NPC
        controller = self.controller
        assert isinstance(controller, LaneFollowController)
        self.sim_obj.follow_closest_lane(True, max_speed=self.controller.max_speed, isLaneChange=True)

    def _apply_apollo_controller(self):
        assert self.type == AgentType.EGO
        start_pos = self.state.forward(-3).position
        transform = comopt.map.SVLTransform(self.simulator.svl_sim)
        start_gps_pos = transform.std_to_gps(start_pos)
        destination_gps_pos = transform.std_to_gps(self.controller.destination)
        self.sim_obj.connect_bridge(self.controller.address, self.controller.bridge_port)
        self.controller.stop_modules(self.controller.enable_modules)
        self.controller.start_modules(self.controller.enable_modules)
        time.sleep(7)
        heading = self.state.heading.r if self.state.heading.r <= math.pi else self.state.heading.r - 2 * math.pi
        self.controller.send_routing_request(start_gps_pos, heading, destination_gps_pos)

    def _apply_record_controller(self):
        assert self.type == AgentType.NPC
        waypoints = []
        ticks = self.controller.trace.ticks
        start_state = self.controller.trace[ticks[0]]
        start_sim_state = self.sim_obj.state
        waypoints.append(
            lgsvl.DriveWaypoint(start_sim_state.position, 10, start_sim_state.rotation, self.controller.delay))
        for i in range(1, len(ticks)):
            pre_t = ticks[i - 1]
            cur_t = ticks[i]
            pre_state = self.controller.trace[ticks[i - 1]]
            cur_state = self.controller.trace[ticks[i]]
            pre_sim_state = svl_configuration.std_state_to_sim(self, self.state.rel_to_abs(start_state.abs_to_rel(pre_state)))
            cur_sim_state = svl_configuration.std_state_to_sim(self, self.state.rel_to_abs(start_state.abs_to_rel(cur_state)))
            speed = max(
                (cur_sim_state.position - pre_sim_state.position).magnitude() / (cur_t - pre_t) * self.controller.scale,
                self.controller.min_speed)
            # waypoints[-1].speed = speed
            waypoints.append(lgsvl.DriveWaypoint(pre_sim_state.position, speed, cur_sim_state.rotation))
            # print(max((cur_sim_state.position - pre_sim_state.position).magnitude() / (cur_t - pre_t) * self.controller.scale, self.controller.min_speed))
        waypoints.append(lgsvl.DriveWaypoint(cur_sim_state.position, 0, cur_sim_state.rotation))

        def print_info(agent, index):
            print('waypoint reached', agent, index)

        self.sim_obj.follow(waypoints, loop=False)
        self.sim_obj.on_waypoint_reached = print_info

    def _update_record_controller(self):
        for id, agent in self.simulator.agents.items():
            if id != self.id and agent.state.position.dis_to(self.state.position) < self.controller.stop_distance:
                logging.info('emergency stop')
                sim_state = self.sim_obj.state
                self.sim_obj.follow([lgsvl.DriveWaypoint(sim_state.position, 0, sim_state.rotation)], False)


class SVLStaticItem(StaticItemBase):
    def __init__(self, simulator, id, item_type, model_name, config=None):
       super().__init__(simulator, id, item_type, model_name, config)
    
    @property
    def state(self):
        sim_state = self.sim_obj.object_state
        return AgentState(
            position=svl_configuration.std_from_sim(sim_state.position),
            heading=Radian(-sim_state.rotation.y / 180 * math.pi),
            velocity=svl_configuration.std_from_sim(sim_state.velocity),
            angular_velocity=-math.radians(sim_state.angular_velocity.y),
            width=self.width,
            length=self.length
        )

    @state.setter
    def state(self, state):
        sim_state = svl_configuration.std_state_to_sim(self, state)
        self.sim_obj.object_state = sim_state

    def _init_cone(self):
        self.sim_obj = self.simulator.svl_sim.controllable_add('TrafficCone')
        self.width = self.length, self.width = svl_configuration.get_size(self.type, self.model_name)


class SVLSimulator(SimulatorBase):
    def __init__(self, addr=svl_configuration.default_simulator_config['addr'], port=svl_configuration.default_simulator_config['port']):
        self._addr = addr
        self._port = port
        super().__init__()
        self._svl_sim = None

    @property
    def svl_sim(self):
        if self._svl_sim is None:
            self._svl_sim = lgsvl.Simulator(self._addr, self._port)
        return self._svl_sim

    @check_type
    def add_agent(self, id: str, agent_type: AgentType, model_name: str, config=None):
        assert not id in self.agents and not id in self.static_items, f'Agent or static item `{id}` has been created.'

        model_info = svl_configuration.model_info(agent_type, model_name)
        default_config = model_info['default_config']
        _config = default_config.copy()
        config = {} if config is None else config
        for k, v in config.items():
            if not k in default_config:
                logging.warning(f'Parameter `{k}` is not available for {agent_type.name} in SVL Simulator.')
            else:
                _config[k] = v

        agent = SVLAgent(self, id, agent_type, model_name, _config)
        self.agents[id] = agent

        return agent

    def add_static_item(self, id, item_type, model_name, config=None):
        assert not id in self.agents and not id in self.static_items, f'Agent or static item `{id}` has been created.'
        model_info = svl_configuration.model_info(item_type, model_name)
        default_config = model_info['default_config']
        _config = default_config.copy()
        config = {} if config is None else config

        for k, v in config.items():
            if not k in default_config:
                logging.warning(f'Parameter `{k}` is not available for {item_type.name} in SVL Simulator.')
            else:
                _config[k] = v

        static_item = SVLStaticItem(self, id, item_type, model_name, _config)
        self.static_items[id] = static_item

        return static_item


    def set_weather(self, weather):
        # print('Setting weather', weather)
        default_weather = svl_configuration.default_environment_config('weather')
        self.svl_sim.weather = lgsvl.WeatherState(
            rain=weather['rain'] if 'rain' in weather else default_weather['rain'],
            fog=weather['fog'] if 'fog' in weather else default_weather['fog'],
            wetness=weather['wetness'] if 'wetness' in weather else default_weather['wetness'],
            cloudiness=weather['cloudiness'] if 'cloudiness' in weather else default_weather['cloudiness'],
            damage=weather['damage'] if 'damage' in weather else default_weather['damage'],
        )

    def set_time_of_day(self, t):
        self.svl_sim.set_time_of_day(t, fixed=True)

    # def _find_signal(self, position):
    #     pos = svl_configuration.std_to_sim(position)
    #     controllable = self.svl_sim.get_controllables()
    #     closest_signal = None
    #     closest_distance = INF
    #     for c in controllable:
    #         if c.type != 'signal':
    #             continue
    #         p = c.transform.position
    #         v = Vector(p.x, p.y, p.z)
    #         dis = v.dis_to_3d(pos)
    #         if dis < closest_distance:
    #             closest_distance = dis
    #             closest_signal = c
    #     if closest_signal is None or closest_distance > MAX_SIGNAL_DISTANCE:
    #         raise NameError(
    #             f'Fail to find signal at {position.str_3d()}, closest: {closest_signal.transform.position}, distance = {closest_distance}')
    #     return closest_signal

    def _get_sim_traffic_light(self, traffic_light):
        sim_traffic_light = self.svl_sim.get_controllable(svl_configuration.std_to_sim(traffic_light.position), 'signal')
        return sim_traffic_light

    def get_traffic_light_init_state(self, traffic_light):
        sim_traffic_light = self._get_sim_traffic_light(traffic_light)
        state = sim_traffic_light.default_control_policy[0]['value']
        if state == 'green':
            return TrafficLightState.GREEN
        elif state == 'red':
            return TrafficLightState.RED
        elif state == 'yellow':
            return TrafficLightState.YELLOW
        else:
            raise Exception(f'Unkown traffic light state {state}')

    def get_traffic_light_state(self, traffic_light):
        sim_traffic_light = self._get_sim_traffic_light(traffic_light)
        state = sim_traffic_light.current_state
        if state == 'green':
            return TrafficLightState.GREEN
        elif state == 'red':
            return TrafficLightState.RED
        elif state == 'yellow':
            return TrafficLightState.YELLOW
        else:
            raise Exception(f'Unkown traffic light state {state}')

    def set_traffic_light(self, traffic_light, init_light_state, trigger_distance, yellow_time=3):
        sim_traffic_light = self._get_sim_traffic_light(traffic_light)
        if isinstance(init_light_state, str):
            init_light_state = TrafficLightState.__members__[init_light_state]
        elif not isinstance(init_light_state, TrafficLightState):
            raise TypeError(f'Error traffic light state value {init_light_state}')
        if init_light_state == TrafficLightState.GREEN:
            sim_traffic_light.control(f'green=1;trigger={trigger_distance};yellow={yellow_time};red=22;green=14;loop')
        elif init_light_state == TrafficLightState.RED:
            sim_traffic_light.control(f'red=1;trigger={trigger_distance};red={yellow_time};green=15;red=21;loop')
        else:
            raise ValueError(f'Unsupport initial color {init_light_state.name}')


    def reset(self):
        super().reset()
        if not self.svl_sim.current_scene is None:
            self.svl_sim.reset()
            # for controllable in self.svl_sim.get_controllables():
            #     if controllable.type == 'signal':
            #         controllable.control('green=1;loop')

    def run(self, time_limit=0.0, time_scale=None):
        self.svl_sim.run(time_limit=time_limit, time_scale=time_scale)

    def load_map(self, map_name):
        scene_name = comopt.map.map_config.sim_name(SimulatorType.SVL, map_name)
        if self.svl_sim.current_scene == scene_name:
            self.reset()
        else:
            self.svl_sim.load(scene_name)
            self.reset()

    def close(self):
        super().close()
        self.svl_sim.close()
        self._svl_sim = None

    def __del__(self):
        if not self.svl_sim is None:
            self.svl_sim.close()
        del self._svl_sim
