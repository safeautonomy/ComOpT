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

from typing import Sequence
from comopt import simulator
from comopt import scenario
from comopt.simulator.common import AgentState
from pickle import NONE
import pickle

from loguru import logger
from comopt.controller.controller import ApolloController, LaneFollowController, SingleDestinationController
from comopt.scenario import AgentDescription, ScenarioDescription, TrafficLightControlDescriptoin
import json
import random
import math
from comopt.map import Map
from comopt.search import Space, StructSpace, NeqSpace
from comopt.simulator import svl_configuration
from comopt.utils import And, retry



class ScenarioInstantiator():
    """For the set of inputs with semantic information contained, compute the quantitative k-projection coverage.
    """
    def __init__(self, scenario_instantiation_parameter_file, jcm_file_path, map_name):
        # Read the mapping file and store it in the internal data structure for later access

        # Opening JSON file
        with open(scenario_instantiation_parameter_file) as f:
            # returns JSON object as a dictionary
            self.scenario_instantiation_parameter = json.load(f)

        with open(jcm_file_path, 'rb') as f:
            self.jcm = pickle.load(f)

        self.SCENARIO_DURATION = 120
        self.SPAWN_DISTANCE = 15
        
        self.map_name = map_name

        self.map = Map('SVL', map_name)

    def instantiate_concrete_scenario_space(self, abstract_scenario, map_instantiation):

        jcm = self.jcm
        map = self.map

        # map_instantiation = self.get_information_from_map(abstract_scenario, jcm, map)
        # map_instantiation_space = self.get_map_information_spaces(abstract_scenario).uniform_sample()
        # print(map_instantiation_space)
        # map_instantiation = map_instantiation_space.uniform_sample()

        # Build concrete scanario file
        concrete_scenario = {}
        # concrete_scenario["map_info"] = {}
        # concrete_scenario["map_info"]["map_name"] = 'map_sanfrancisco' # lgsvl.wise.DefaultAssets.map_sanfrancisco
        # concrete_scenario["map_info"]["hd_map_name"] = "San Francisco"

        concrete_scenario['map_name'] = self.map_name

        self.map = Map('SVL', concrete_scenario['map_name'])

        concrete_scenario["ego_vehicle"] = {}
        # concrete_scenario["ego_vehicle"]["start_lane"] = map_instantiation["ego_start"] 

        concrete_scenario["ego_vehicle"]["start_lane"] = map_instantiation["start_lane"]
        if abstract_scenario["ego_vehicle_movement_pattern"] == "drive_straight":
            concrete_scenario["ego_vehicle"]["spawn"] = map.lanes[map_instantiation["start_lane"]].get_spawns(self.SPAWN_DISTANCE, end_offset=20)[0]
        else:
            available_spawns = set(filter(
                lambda spawn: spawn.s < spawn.lane.length - self._get_brake_distance_by_v(self.scenario_instantiation_parameter['ego_vehicle_initial_speed'][abstract_scenario["ego_vehicle_initial_speed"]][1]), 
                map.lanes[map_instantiation["start_lane"]].get_spawns(self.SPAWN_DISTANCE, end_offset=20)
            ))
            concrete_scenario["ego_vehicle"]["spawn"] = Space(available_spawns)


        # concrete_scenario["ego_vehicle"]["initial_location"] = map_instantiation["ego_vehicle"]["initial_location"]
        # concrete_scenario["ego_vehicle"]["initial_rotation"] = map_instantiation["ego_vehicle"]["initial_rotation"]
        # The speed is set by selecting the range
        # speed = sum(self.scenario_instantiation_parameter["ego_vehicle_initial_speed"][abstract_scenario["ego_vehicle_initial_speed"]])/2
        speed = Space(self.scenario_instantiation_parameter['ego_vehicle_initial_speed'][abstract_scenario["ego_vehicle_initial_speed"]])
        concrete_scenario["ego_vehicle"]["initial_speed"] = speed

        # The waypoint is based on the high-level motion primitive (e.g., turn_left) 
        # concrete_scenario["ego_vehicle"]["waypoints"] = map_instantiation["ego_vehicle"]["waypoints"][abstract_scenario["ego_vehicle_movement_pattern"]] 
        
        # print(abstract_scenario["ego_vehicle_movement_pattern"])
        concrete_scenario["ego_vehicle"]["end_lane"] = map_instantiation['end_lane'] # map_instantiation[abstract_scenario["ego_vehicle_movement_pattern"]] 

        concrete_scenario ["other_road_segments"] = map_instantiation["other_road_segments"]

        # Other road participants to be controlled by the simulator
        # concrete_scenario["number_of_vehicles"] =  int(sum(self.scenario_instantiation_parameter["density_of_vehicles"][abstract_scenario["density_of_vehicles"]])/2)
        concrete_scenario["number_of_vehicles"] = Space(self.scenario_instantiation_parameter["density_of_vehicles"][abstract_scenario["density_of_vehicles"]])
        # concrete_scenario["number_of_crossing_pedestrians"] =  int(sum(self.scenario_instantiation_parameter["density_of_crossing_pedestrians"][abstract_scenario["density_of_crossing_pedestrians"]])/2)
        concrete_scenario["max_number_of_crossing_pedestrians"] = \
            self.scenario_instantiation_parameter["density_of_crossing_pedestrians"][abstract_scenario["density_of_crossing_pedestrians"]][1]

        concrete_scenario["max_number_of_vehicles"] =  int(self.scenario_instantiation_parameter["density_of_vehicles"][abstract_scenario["density_of_vehicles"]][1])
        # concrete_scenario["front_vehicle"] = True if abstract_scenario["front_vehicle"] == "True" else False
        # concrete_scenario["oncoming_vehicle"] = True if abstract_scenario["oncoming_vehicle"] == "True" else False

        concrete_scenario["npc_vehicle_info"] = {}
        concrete_scenario["npc_vehicle_info"]["speed"] = Space(*([self.scenario_instantiation_parameter["npc_vehicle_info"]["speed"]] * concrete_scenario["max_number_of_vehicles"]), sequence = True)
        concrete_scenario["npc_vehicle_info"]["model"] = Space(*([set(svl_configuration.models('NPC'))] * concrete_scenario["max_number_of_vehicles"]), sequence = True)
        concrete_scenario["npc_vehicle_info"]["config"] = Space(*([StructSpace({'color': {'R': Space(0.0, 1.0), 'G': Space(0.0, 1.0), 'B': Space(0.0, 1.0)}})] * concrete_scenario['max_number_of_vehicles']))

        spawns = set()
        for lane_id in map_instantiation["other_road_segments"]:
            spawns |= set(map.lanes[lane_id].get_spawns(self.SPAWN_DISTANCE))

        concrete_scenario["npc_vehicle_info"]["spawn"] = NeqSpace(spawns, concrete_scenario["max_number_of_vehicles"])

        

        concrete_scenario["crosswalk"] = {}
        concrete_scenario["crosswalk"]["entrance"] = True if abstract_scenario["entrance_crosswalk"] == 'True' else False
        concrete_scenario["crosswalk"]["exit"] = True if abstract_scenario["exit_crosswalk"] == 'True' else False

        if abstract_scenario['entrance_crosswalk'] or abstract_scenario['exit_crosswalk']:
            concrete_scenario["npc_pedestrian_info"] = {}

        if concrete_scenario["crosswalk"]["entrance"]:
            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"] = {}
            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["number"] = \
                self.scenario_instantiation_parameter["density_of_crossing_pedestrians"][abstract_scenario["density_of_crossing_pedestrians"]][0]
                # Space(self.scenario_instantiation_parameter["density_of_crossing_pedestrians"][abstract_scenario["density_of_crossing_pedestrians"]])
                
            
            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["speed"] = \
                Space(*[self.scenario_instantiation_parameter["npc_pedestrian_info"]["speed"]] * concrete_scenario["max_number_of_crossing_pedestrians"], sequence = True)

            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["model"] = \
                Space(*[set(svl_configuration.models('PEDESTRIAN'))] * concrete_scenario["max_number_of_crossing_pedestrians"], sequence = True)
            
            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["trigger_distance"] = \
                Space(*([self.scenario_instantiation_parameter["npc_pedestrian_info"]["trigger_distance"]] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence=True)
            
            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["start_side"] = \
                Space(*([{'left', 'right'}] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence = True)
            
            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["start_pos"] = \
                Space(*([[0.0, 1.0]] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence=True)

            concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["end_pos"] = \
                Space(*([[0.0, 1.0]] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence=True)

        if concrete_scenario["crosswalk"]["exit"]:
            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"] = {}
            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["number"] = \
                Space(self.scenario_instantiation_parameter["density_of_crossing_pedestrians"][abstract_scenario["density_of_crossing_pedestrians"]])

            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["speed"] = \
                Space(*[self.scenario_instantiation_parameter["npc_pedestrian_info"]["speed"]] * concrete_scenario["max_number_of_crossing_pedestrians"], sequence=True)

            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["model"] = \
                Space(*[set(svl_configuration.models('PEDESTRIAN'))] * concrete_scenario["max_number_of_crossing_pedestrians"], sequence = True)

            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["trigger_distance"] = \
                Space(*([self.scenario_instantiation_parameter["npc_pedestrian_info"]["trigger_distance"]] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence=True)
            
            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["start_side"] = \
                Space(*([{'left', 'right'}] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence=True)

            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["start_pos"] = \
                Space(*([[0.0, 1.0]] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence=True)

            concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["end_pos"] = \
                Space(*([[0.0, 1.0]] * concrete_scenario["max_number_of_crossing_pedestrians"]), sequence=True)


        concrete_scenario["has_traffic_light"] = True if abstract_scenario["has_traffic_light"] == 'True' else False

        if concrete_scenario["has_traffic_light"]:
            concrete_scenario["traffic_light_info"] = {}
            concrete_scenario["traffic_light_info"]["trigger_distance"] = Space(self.scenario_instantiation_parameter["traffic_light_trigger_distance"][abstract_scenario["traffic_light_trigger_distance"]])
            concrete_scenario["traffic_light_info"]["initial_color"] = abstract_scenario["traffic_light_initial_color"]

        concrete_scenario["junction_info"] = map_instantiation["junction_info"]

        # Weather and other environmental conditions
        concrete_scenario["environment_condition"] = {}
        concrete_scenario["environment_condition"]["weather"]={}
        weather_parameters = self.scenario_instantiation_parameter["weather"][abstract_scenario["weather"]]


        cloudiness = Space(weather_parameters['cloudiness'])
        rain = Space(weather_parameters['rain'])
        wetness = Space(weather_parameters['wetness'])
        fog = Space(weather_parameters['fog'])


        concrete_scenario["environment_condition"]["weather"]["cloudiness"] = cloudiness
        concrete_scenario["environment_condition"]["weather"]["rain"] = rain
        concrete_scenario["environment_condition"]["weather"]["wetness"] = wetness
        concrete_scenario["environment_condition"]["weather"]["fog"] = fog

        concrete_scenario["environment_condition"]["road_damage"] = Space(self.scenario_instantiation_parameter["road_damage"][abstract_scenario["road_damage"]])

        # Simulator options
        concrete_scenario["simulation_options"] = {}
        concrete_scenario["simulation_options"]["duration"] = self.SCENARIO_DURATION


        return StructSpace(concrete_scenario)

    def get_start_lane_crosswalk(self, lane_id, map):
        lane = map.lanes[lane_id]
        for cw in lane.crosswalks:
            if cw.cover_range.end_s > lane.length - 15:
                return cw.crosswalk
        for l in lane.successors:
            for cw in l.crosswalks:
                if cw.cover_range.start_s < 10:
                    return cw.crosswalk
        return None

    def get_end_lane_crosswalk(self, lane_id, map):
        lane = map.lanes[lane_id]
        for cw in lane.crosswalks:
            if cw.cover_range.start_s < 15:
                return cw.crosswalk
        for l in lane.successors:
            for cw in l.crosswalks:
                if cw.cover_range.end_s > l.length - 10:
                    return cw.crosswalk
        return None

    # def get_information_from_map(self, abstract_scenario, jcm, map : Map):
    def get_map_information_spaces(self, abstract_scenario):
        # Call the external function to get all required info needed. 

        jcm = self.jcm
        map = self.map

        instantiated_road_segment_info = {}
        instantiated_road_segment_info["other_road_segments"] = []
        ego_vehicle_starting_lane_id = None
        ego_vehicle_ending_lane_id = None

        if abstract_scenario['has_traffic_light'] == 'True':
            ego_start_lane_min_dis = self.scenario_instantiation_parameter["traffic_light_trigger_distance"][abstract_scenario["traffic_light_trigger_distance"]][1]
        else:
            ego_start_lane_min_dis = 0

        def check_start_lane(lane_id):
            lane = map.lanes[lane_id]
            if lane.length < ego_start_lane_min_dis: return False
            if abstract_scenario["entrance_crosswalk"] == 'True':
                if self.get_start_lane_crosswalk(lane_id, map) is None:
                    return False
            if abstract_scenario["has_traffic_light"] == 'True':
                if not len(lane.traffic_lights) + sum([len(l.traffic_lights) for l in lane.successors]) > 0:
                    return False
            elif abstract_scenario["has_traffic_light"] == 'False':
                if not len(lane.traffic_lights) + sum([len(l.traffic_lights) for l in lane.successors]) == 0:
                    return False
            return True

        def check_end_lane(lane_id):
            if abstract_scenario["exit_crosswalk"] == 'True':
                if self.get_end_lane_crosswalk(lane_id, map) is None:
                    return False
            return True

        # The lane index in the constraint should subtract 1 to get the index in the map
        # total_lanes_in_one_side = math.ceil(int(abstract_scenario["total_lanes"]) / 2)
        # current_lane_index = int(abstract_scenario["current_lane"]) - 1

        # print("total_lanes in one side:" + str(total_lanes_in_one_side))
        # print("current_lane_index:" + str(current_lane_index))
        # First decide on the junction to be taken, depending on the shape.

        total_lanes = int(abstract_scenario["total_lanes"])

        if abstract_scenario["road_structure"] == "T_way_intersection": 

            road_search_token = self.scenario_instantiation_parameter["road_structure"]["T_way_intersection"]
            # instantiated_road_segment_info["road_search_token"] = road_search_token

            available_junction_spaces = set()

            for junction in jcm[road_search_token]:
                info = junction.important_lanes()['case_no_forward']
                local_space = StructSpace()
                # if len(info['self']['entrance']) < total_lanes: continue

                start_lanes = {lane_id for lane_id in info['self']['entrance'] if check_start_lane(lane_id)}
                if len(start_lanes) == 0: 
                    continue                  

                local_space['start_lane'] = Space(start_lanes)
                if abstract_scenario['ego_vehicle_movement_pattern'] == 'turn_left':
                    end_lanes = {lane_id for lane_id in info['left']['exit'] if check_end_lane(lane_id)}
                    if len(end_lanes) == 0: 
                        continue
                    local_space['end_lane'] = Space(end_lanes)
                elif abstract_scenario['ego_vehicle_movement_pattern'] == 'turn_right':
                    end_lanes = {lane_id for lane_id in info['right']['exit'] if check_end_lane(lane_id)}
                    if len(end_lanes) == 0: continue
                    local_space['end_lane'] = Space(end_lanes)
                local_space['other_road_segments'] = []
                local_space['other_road_segments'].extend(info['self']['entrance'])
                local_space['other_road_segments'].extend(info['self']['exit'])
                local_space['other_road_segments'].extend(info['left']['entrance'])
                local_space['other_road_segments'].extend(info['right']['entrance'])

                local_space['junction_info'] = info

                available_junction_spaces.add(local_space)

            assert len(available_junction_spaces) > 0
            return Space(available_junction_spaces)


        elif abstract_scenario["road_structure"] == "4_way_intersection": 

            road_search_token = self.scenario_instantiation_parameter["road_structure"]["4_way_intersection"]
            # instantiated_road_segment_info["road_search_token"] = road_search_token

            available_junction_spaces = set()
            for junction in jcm[road_search_token]:
                infos = junction.important_lanes()
                for info in infos.values():
                    local_space = StructSpace()
                    # if len(info['self']['entrance']) < total_lanes: continue
                    start_lanes = {lane_id for lane_id in info['self']['entrance'] if check_start_lane(lane_id)}
                    if len(start_lanes) == 0: 
                        continue                  

                    local_space['start_lane'] = Space(start_lanes)
                    if abstract_scenario['ego_vehicle_movement_pattern'] == 'turn_left':
                        end_lanes = {lane_id for lane_id in info['left']['exit'] if check_end_lane(lane_id)}
                        if len(end_lanes) == 0:
                            continue
                        local_space['end_lane'] = Space(end_lanes)
                    elif abstract_scenario['ego_vehicle_movement_pattern'] == 'turn_right':
                        end_lanes = {lane_id for lane_id in info['right']['exit'] if check_end_lane(lane_id)}
                        if len(end_lanes) == 0: 
                            continue
                        local_space['end_lane'] = Space(end_lanes)
                    elif abstract_scenario['ego_vehicle_movement_pattern'] == 'drive_straight':
                        end_lanes = {lane_id for lane_id in info['forward']['exit'] if check_end_lane(lane_id)}
                        if len(end_lanes) == 0: 
                            continue
                        local_space['end_lane'] = Space(end_lanes)
                    local_space['other_road_segments'] = []
                    local_space['other_road_segments'].extend(info['self']['entrance'])
                    local_space['other_road_segments'].extend(info['self']['exit'])
                    local_space['other_road_segments'].extend(info['left']['entrance'])
                    local_space['other_road_segments'].extend(info['right']['entrance'])
                    local_space['other_road_segments'].extend(info['forward']['entrance'])

                    local_space['junction_info'] = info

                    available_junction_spaces.add(local_space)
                
            assert len(available_junction_spaces) > 0
            return Space(available_junction_spaces)



        elif abstract_scenario["road_structure"] == "straight_lane": 
            
            road_search_token = self.scenario_instantiation_parameter["road_structure"]["straight_lane"]

            available_junction_spaces = set()
            for junction in jcm[road_search_token]:
                infos = junction.important_lanes()
                for info in infos.values():
                    
                    # if len(info['self']['entrance']) < total_lanes: continue
                    local_space = StructSpace()
                    
                    start_lanes = {lane_id for lane_id in info['self']['entrance'] if check_start_lane(lane_id)}
                    if len(start_lanes) == 0: continue                  
                    local_space['start_lane'] = Space(start_lanes)

                    end_lanes = {lane_id for lane_id in info['other']['exit'] if check_end_lane(lane_id)}
                    if len(end_lanes) == 0: continue
                    local_space['end_lane'] = Space(end_lanes)    
                    
                    local_space['other_road_segments'] = []
                    local_space['other_road_segments'].extend(info['self']['entrance'])
                    local_space['other_road_segments'].extend(info['self']['exit'])
                    local_space['other_road_segments'].extend(info['other']['exit'])
                    local_space['other_road_segments'].extend(info['other']['entrance'])

                    local_space['junction_info'] = info

                    available_junction_spaces.add(local_space)

            assert len(available_junction_spaces) > 0
            return Space(available_junction_spaces)


        else:
            raise NotImplementedError(str(abstract_scenario["road_structure"]) + " is not supported")


    def get_traffic_light(self, lane_id):
        lane = self.map.lanes[lane_id]
        for tl in lane.traffic_lights:
            return tl
        for su_lane in lane.successors:
            for tl in su_lane.traffic_lights:
                return tl

    def gen_scenario_description(self, concrete_scenario):
        if self.map is None:
            self.map = Map('SVL', concrete_scenario['map_name'])
        map = self.map

        scenario_description = ScenarioDescription('SVL', concrete_scenario['map_name'], concrete_scenario['simulation_options']['duration'])
        
        scenario_description.add_agent(AgentDescription(
            'aut',
            'EGO',
            'Lincoln2017MKZ',
            concrete_scenario['ego_vehicle']['spawn'].get_state(concrete_scenario['ego_vehicle']['initial_speed']),    
            ApolloController(map.lanes[concrete_scenario['ego_vehicle']['end_lane']].get_normal_state_by_s(-5).position)
        ))

        scenario_description.weather['rain'] = concrete_scenario['environment_condition']['weather']['rain']
        scenario_description.weather['cloudiness'] = concrete_scenario['environment_condition']['weather']['cloudiness']
        scenario_description.weather['wetness'] = concrete_scenario['environment_condition']['weather']['wetness']
        scenario_description.weather['fog'] = concrete_scenario['environment_condition']['weather']['fog']
    
        scenario_description.weather['damage'] = concrete_scenario['environment_condition']['road_damage']

        for i in range(concrete_scenario['number_of_vehicles']):
            scenario_description.add_agent(AgentDescription(
                f'npc{i}',
                'NPC',
                concrete_scenario['npc_vehicle_info']['model'][i],
                concrete_scenario['npc_vehicle_info']['spawn'][i].get_state(),
                LaneFollowController(max_speed=concrete_scenario['npc_vehicle_info']['speed'][i]),
                config = concrete_scenario['npc_vehicle_info']['config'][i]
            ))
        
        
        offset = 0
        if concrete_scenario["crosswalk"]["entrance"]:
            crosswalk = self.get_start_lane_crosswalk(concrete_scenario['ego_vehicle']['start_lane'], self.map)
            for i in range(concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["number"]):
                start_pos = concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["start_pos"][i]
                end_pos = concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["end_pos"][i]
                start_side = concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["start_side"][i]
                speed = concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["speed"][i]
                trigger_distance = concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["trigger_distance"][i]
                model = concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["model"][i]
                if start_side == 'left':
                    start_seg = crosswalk.sides.left
                    end_seg = crosswalk.sides.right
                else:
                    start_seg = crosswalk.sides.right
                    end_seg = crosswalk.sides.left
                scenario_description.add_agent(AgentDescription(
                    f'ped{i}',
                    'PEDESTRIAN',
                    model,
                    AgentState(start_seg.lerp(start_pos)),
                    SingleDestinationController(end_seg.lerp(end_pos), speed, trigger_distance)
                ))
            offset += concrete_scenario["npc_pedestrian_info"]["entrance_crosswalk"]["number"]
        
        if concrete_scenario["crosswalk"]["exit"]:
            crosswalk = self.get_end_lane_crosswalk(concrete_scenario['ego_vehicle']['end_lane'], self.map)
            for i in range(concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["number"]):
                start_pos = concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["start_pos"][i]
                end_pos = concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["end_pos"][i]
                start_side = concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["start_side"][i]
                speed = concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["speed"][i]
                trigger_distance = concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["trigger_distance"][i]
                model = concrete_scenario["npc_pedestrian_info"]["exit_crosswalk"]["model"][i]
                if start_side == 'left':
                    start_seg = crosswalk.sides.left
                    end_seg = crosswalk.sides.right
                else:
                    start_seg = crosswalk.sides.right
                    end_seg = crosswalk.sides.left
                scenario_description.add_agent(AgentDescription(
                    f'ped{offset + i}',
                    'PEDESTRIAN',
                    model,
                    AgentState(start_seg.lerp(start_pos)),
                    SingleDestinationController(end_seg.lerp(end_pos), speed, trigger_distance)
                ))
        

        if concrete_scenario["has_traffic_light"]:
            start_lane = map.lanes[concrete_scenario["ego_vehicle"]["start_lane"]]
            traffic_lights = set()

            simulator = scenario_description.simulator_obj
            for lane_id in concrete_scenario["other_road_segments"]:
                traffic_lights.add(self.get_traffic_light(lane_id))
            if None in traffic_lights:
                traffic_lights.remove(None)
            
            @retry("Exception occurs when load_map")
            def load_map():
                simulator.load_map(self.map_name)
            
            load_map()
            
            ego_light_state = simulator.get_traffic_light_init_state(self.get_traffic_light(start_lane.id))
            
            scenario_description.add_record_traffic_light(self.get_traffic_light(start_lane.id))
            
            opposite_color = {
                'green': 'red',
                'red': 'green'
            }
            
            trigger_position = start_lane.get_normal_state_by_s(-concrete_scenario["traffic_light_info"]["trigger_distance"]).position
            for traffic_light in traffic_lights:
                color = concrete_scenario["traffic_light_info"]["initial_color"].upper() if simulator.get_traffic_light_init_state(traffic_light) == ego_light_state else opposite_color[concrete_scenario["traffic_light_info"]["initial_color"]].upper()
                scenario_description.add_traffic_light_control(TrafficLightControlDescriptoin(
                    traffic_light.id, 
                    color,
                    traffic_light.dis_to_ground(trigger_position)
                ))
            scenario_description._drop_simulator()
        

        return scenario_description

    def _get_brake_distance_by_v(self, v):
        return 0.5 * v ** 2 / 6.94

    def _check_instatiated_scenario(self, instantiated_scenario):
        min_distance = self._get_brake_distance_by_v(instantiated_scenario["ego_vehicle"]["initial_speed"])
        for spawn in instantiated_scenario["npc_vehicle_info"]["spawn"]:
            if spawn.lane.id == instantiated_scenario["ego_vehicle"]["start_lane"] and \
                instantiated_scenario["ego_vehicle"]["spawn"].s <= spawn.s < instantiated_scenario["ego_vehicle"]["spawn"].s + min_distance + 0.5:
                return False
        return True

    def _sample_instantiated_scenario(self, instantiated_scenario_space):
        instantiated_scenario = instantiated_scenario_space.uniform_sample()
        while self._check_instatiated_scenario(instantiated_scenario) == False:
            instantiated_scenario = instantiated_scenario_space.uniform_sample()
        return instantiated_scenario

    def sample_from_abstract_scenario(self, abstract_scenario):
        map_instantiation_spaces = self.get_map_information_spaces(abstract_scenario)
        map_instantiation = map_instantiation_spaces.uniform_sample().uniform_sample()
        instantiated_scenario_space = self.instantiate_concrete_scenario_space(abstract_scenario, map_instantiation)
        instantiated_scenario = self._sample_instantiated_scenario(instantiated_scenario_space)

        scenario_runner = self.gen_scenario_description(instantiated_scenario)
        while not scenario_runner.available:
            instantiated_scenario = self._sample_instantiated_scenario(instantiated_scenario_space)
            scenario_runner = self.gen_scenario_description(instantiated_scenario)
        return scenario_runner







