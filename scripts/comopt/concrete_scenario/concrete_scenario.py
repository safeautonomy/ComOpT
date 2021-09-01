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

import lgsvl
from environs import Env
import math, json, pickle
from ..utility.dreamview import Connection

env = Env()

class ScenarioRunner:
    def __init__(self, scenario:dict, enable_dreamview=True):
        self.sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1"), env.int("LGSVL__SIMULATOR_PORT", 8181))
        self.scenario = scenario
        self.enable_dreamview = enable_dreamview

    def _on_collision(self, agent1, agent2, contact):
        print('collision', type(agent1), type(agent2), contact)
        if type(agent2) == lgsvl.agent.Pedestrian and agent1.state.velocity.magnitude() < 0.2:
            return
        raise lgsvl.evaluator.TestException("Ego collided with {}".format(agent2))

    def _on_waypoint(self, agent, index):
        print("Agent {} reached Waypoint {}".format(agent, index))

    def _add_ego(self, scenario):
        state_ego = self._parse_init_state(scenario["ego_vehicle"])
        sim_vehicle_name = env.str("LGSVL__SIMULATION_VEHICLE_NAME", "2e9095fa-c9b9-4f3f-8d7d-65fa2bb03921")
        ego = self.sim.add_agent(sim_vehicle_name, lgsvl.AgentType.EGO, state_ego)
        ego.on_collision(self._on_collision)
        ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1"), env.int("LGSVL__AUTOPILOT_0_BRIDGE_PORT", 9090))
        if self.enable_dreamview:
            self._set_dreamview(ego, scenario)
        self.ego = ego
        return ego

    def _set_pedestrian(self, agent_obj, agent):
        wp_to_be_fed_in_sim = []
        for wp in  agent["waypoints"]:
            waypoint = wp["position"]
            stopping_time = wp["stopping_time"]

            wp_to_be_fed_in_sim.append(lgsvl.WalkWaypoint(lgsvl.Vector(waypoint[0], waypoint[1], waypoint[2]), stopping_time))

        # agent_obj.on_collision(self.on_collision)
        # agent_obj.on_waypoint_reached(self._on_waypoint)

        if  agent["waypoints"]:
            # If the waypoint is not empty, execute
            agent_obj.follow(wp_to_be_fed_in_sim, False)

    def _set_follow_waypoints(self, agent_obj, agent):
        wp_to_be_fed_in_sim = []
        for wp in  agent["waypoints"]:
            waypoint = wp["position"]
            angle = wp["angle"]
            speed = wp["speed"]
            wp_to_be_fed_in_sim.append(lgsvl.DriveWaypoint(lgsvl.Vector(waypoint[0], waypoint[1], waypoint[2]), speed, lgsvl.Vector(angle[0], angle[1], angle[2]), 0, False, 0) )

        if  agent["waypoints"]:
            # If the waypoint is not empty, execute
            agent_obj.follow(wp_to_be_fed_in_sim, False)

    def _set_follow_closest_lane(self, agent_obj, agent):
        agent_obj.follow_closest_lane(True, max_speed = agent["max_speed"])

    def _set_follow_waypoints_in_file(self, agent_obj, agent):
        with open(agent["file"], "rb") as f:
            waypoints_from_file = pickle.load(f)
        waypoints = []
        for waypoint in waypoints_from_file:
            waypoint.speed = max(waypoint.speed, 0.1)
            waypoints.append(waypoint)
        agent_obj.follow(waypoints, False)

    def _set_npc(self, agent_obj, agent):
        if not "control" in agent or agent["control"] == "follow_waypoints":
            self._set_follow_waypoints(agent_obj, agent)
        elif agent["control"] == "follow_closest_lane":
            self._set_follow_closest_lane(agent_obj, agent)
        elif agent["control"] == "follow_waypoints_in_file":
            self._set_follow_waypoints_in_file(agent_obj, agent)
        
    def _parse_vector(self, vector):
        return lgsvl.Vector(vector[0], vector[1], vector[2])

    def _parse_init_state(self, agent):
        state = lgsvl.AgentState()
        
        if not "control" in agent or agent["control"] != "follow_waypoints_in_file":
            state.transform.position = self._parse_vector(agent["initial_location"])
            state.transform.rotation = self._parse_vector(agent["initial_rotation"])
            # state.speed = agent["initial_speed"]
        else:
            with open(agent["file"], "rb") as f:
                waypoints_from_file = pickle.load(f)
            state.transform.position = waypoints_from_file[0].position
            state.transform.rotation = waypoints_from_file[0].angle
        return state

    def _add_agent(self, agent):
        agent_state = self._parse_init_state(agent)
        if agent["type"] == "pedestrian":
            agent_obj = self.sim.add_agent("Pamela", lgsvl.AgentType.PEDESTRIAN, agent_state)
            self._set_pedestrian(agent_obj, agent)
        elif agent["type"] == "vehicle":
            agent_obj = self.sim.add_agent(agent["detailed_type"], lgsvl.AgentType.NPC, agent_state)
            self._set_npc(agent_obj, agent)
        # agent_obj.on_waypoint_reached(self._on_waypoint)

    def _add_weather(self, scenario):
        if "environment_conditions" in scenario.keys() :

            w = self.sim.weather 
            w.rain = scenario["environment_conditions"]["weather"]["rain"]     
            w.cloudiness =  scenario["environment_conditions"]["weather"]["cloudiness"]   
            w.wetness = scenario["environment_conditions"]["weather"]["wetness"]   
            self.sim.weather = w
    

    def _is_close_by(self, point_a,  point_b, dist):
        if (point_a.x - point_b.x) **2 +  (point_a.y - point_b.y) **2  + (point_a.z - point_b.z) **2 <= dist**2 :
            return True
        else:
            return False

    def _set_controllables(self, scenario):
        controllables = self.sim.get_controllables("signal")
        for control_in_scenario in scenario["signals_to_be_controlled"]:
            reference_point = control_in_scenario["reference_point"]
            radius = control_in_scenario["radius"]
            for c in controllables:
                if (self._is_close_by(c.transform.position, lgsvl.Vector(reference_point[0], reference_point[1], reference_point[2]), radius)):
                    # Control this traffic light with a new control policy
                    c.control(control_in_scenario["control_policy"])

    def _set_dreamview(self, ego, scenario):
        modules = env.list("LGSVL__AUTOPILOT_0_VEHICLE_MODULES",[
                'Localization',
                'Transform',
                'Routing',
                'Prediction',
                'Planning',
                'Control'
            ], subcast=str)
        waypoints = scenario["ego_vehicle"]["waypoints"]
        dv = Connection(self.sim, ego, ip=env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1"), port=env.str("LGSVL__AUTOPILOT_0_DREAMVIEW_PORT", "8888"))
        dv.set_setup_mode(env.str("LGSVL__AUTOPILOT_0_VEHICLE_MODE", "Mkz Lgsvl"))
        dv.set_hd_map(scenario["map_info"]["hd_map_name"])
        dv.set_vehicle(env.str("LGSVL__AUTOPILOT_0_VEHICLE_NAME", "Lincoln2017MKZ LGSVL"))
        if isinstance(waypoints[0], int) or isinstance(waypoints[0], float):
            dv.enable_apollo(waypoints[0], waypoints[2], modules)
        else:
            dv.enable_apollo(
                [waypoint[0] for waypoint in waypoints], 
                [waypoint[2] for waypoint in waypoints], 
                modules
            )
        self.dv = dv

    def exec(self, save_file=None):

        #self.sim.reset()

        if self.sim.current_scene == self.scenario["map_info"]["map_name"]:
            self.sim.reset()
        else:
            self.sim.load(self.scenario["map_info"]["map_name"])
        
        self._add_ego(self.scenario)

        for agent in self.scenario["other_road_participants"]:
            self._add_agent(agent)

        self._set_controllables(self.scenario)

        self._add_weather(self.scenario)

        simulation_time = self.scenario["simulation_options"]["duration"]
        try:
            if save_file is None:
                self.sim.run(simulation_time)
            else:
                time_step = self.scenario["simulation_options"]["resolution"]
                time_passed = 0
                ego_record = []
                while time_passed < simulation_time:
                    ego_record.append(lgsvl.DriveWaypoint(
                        position=self.ego.state.position, 
                        speed=max(self.ego.state.velocity.magnitude(), 0.1),
                        angle=self.ego.state.rotation
                    ))
                    self.sim.run(time_step)
                    time_passed += time_step
                with open(save_file, "wb") as f:
                    pickle.dump(ego_record, f)
        finally:
            self.dv.disable_apollo()
        
