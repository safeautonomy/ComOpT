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

from comopt.analyzer.analyzer import TraceAnalyzer
from os import times
from comopt.simulator.common import AgentType
from comopt.search import MutAgentInfo, SingleStateSearch
from comopt.analyzer.behavior import *
from queue import PriorityQueue

class SingleStateProblem:
    def __init__(self, single_state_search, priority, name=''):
        self.single_state_search = single_state_search
        self.priority = priority
        self.name = name

    def search(self):
        return self.single_state_search.search()
    
    def __lt__(self, o):
        return self.priority > o.priority

    def __eq__(self, o):
        return self.priority == o.priority

    def __le__(self, o):
        return self.priority >= o.priority

class SingleStateSearchGenerator:

    def __init__(self, base_scenario, analyzer, controller_settings, mut_agent_id, aut_id, time_limit):
        self.base_scenario = base_scenario
        self.analyzer = analyzer
        self.controller_settings = controller_settings
        self.mut_agent_id = mut_agent_id
        self.aut_id = aut_id
        self.time_limit = time_limit

    def generate(self):
        ret = []
        for behavior in self.analyzer.behaviors:
            problems = tuple()
            if behavior.type == BehaviorType.FOLLOW_LANE:
                problems = self._gen_follow_lane(behavior)
                # pass
            elif behavior.type == BehaviorType.CHANGE_LANE:
                problems = self._gen_change_lane(behavior)
            elif behavior.type == BehaviorType.DEPARTURE:
                problems = self._gen_departure(behavior)
            elif behavior.type == BehaviorType.OVERTAKING:
                problems = self._gen_overtaking(behavior)
            # elif behavior.type == BehaviorType.TURN_RIGHT:
            #     problems = self._gen_turn_right(behavior)

            for problem in problems:
                ret.append(SingleStateProblem(problem, behavior.danger))
        return ret

    def _gen_follow_lane(self, behavior):
        controllers = self.controller_settings[BehaviorType.FOLLOW_LANE] if BehaviorType.FOLLOW_LANE in self.controller_settings else self.controller_settings['default']
        ret = []
        behavior_start = behavior.trace[behavior.trace.ticks[0]]
        for controller in controllers:
            for keypoint in behavior.keypoints:
                mut_agent_lane = keypoint.item.obj
                predicted_s = keypoint.item.s - controller.predict_travel_distance_by_t(keypoint.time)
                if behavior_start.lane_info.center.obj == mut_agent_lane and behavior_start.lane_info.center.s > predicted_s - 5:
                    continue
                ret.append(SingleStateProblem(SingleStateSearch(
                    self.base_scenario,
                    MutAgentInfo(
                        self.mut_agent_id,
                        AgentType.NPC,
                        'Sedan',
                        controller,
                        mut_agent_lane,
                        predicted_s - 5,
                        predicted_s + 5
                    ), mut_agent_lane, self.time_limit, self.aut_id, max_iter=1
                ), behavior.danger))
        return ret

    def _gen_change_lane(self, behavior):
        controllers = self.controller_settings[BehaviorType.CHANGE_LANE] if BehaviorType.CHANGE_LANE in self.controller_settings else self.controller_settings['default']
        ret = []
        for controller in controllers:
            for keypoint in behavior.keypoints:
                mut_agent_lane = keypoint.item.obj
                predicted_s = keypoint.item.s - controller.predict_travel_distance_by_t(keypoint.time)
                ret.append(SingleStateProblem(SingleStateSearch(
                    self.base_scenario,
                    MutAgentInfo(
                        self.mut_agent_id,
                        AgentType.NPC,
                        'Sedan',
                        controller,
                        mut_agent_lane,
                        predicted_s - 10,
                        predicted_s + 10
                    ), mut_agent_lane, self.time_limit, self.aut_id
                ), behavior.danger, 'Change Lane Problem'))
        return ret

    def _gen_departure(self, behavior):
        controllers = self.controller_settings[BehaviorType.DEPARTURE] if BehaviorType.DEPARTURE in self.controller_settings else self.controller_settings['default']
        ret = []
        start_lane = behavior.trace[behavior.trace.ticks[0]].lane_info.center.obj
        for controller in controllers:
            for keypoint in behavior.keypoints:
                
                
                mut_agent_lane = keypoint.item.obj
                try:
                    predicted_s = keypoint.item.s - controller.predict_travel_distance_by_t(keypoint.time) + 5 # - 40
                except:
                    continue
                reverse = mut_agent_lane in start_lane.left_reverse_neighbors or mut_agent_lane in start_lane.right_reverse_neighbors
                ret.append(SingleStateProblem(SingleStateSearch(
                    self.base_scenario,
                    MutAgentInfo(
                        self.mut_agent_id,
                        AgentType.NPC,
                        'Sedan',
                        controller,
                        mut_agent_lane,
                        predicted_s - 10,
                        predicted_s + 10
                    ), mut_agent_lane, self.time_limit, self.aut_id, reverse
                ), behavior.danger))
        return ret

    def _gen_overtaking(self, behavior):
        controllers = self.controller_settings[BehaviorType.OVERTAKING] if BehaviorType.OVERTAKING in self.controller_settings else self.controller_settings['default']
        ret = []
        for controller in controllers:
            for keypoint in behavior.keypoints:
                mut_agent_lane = keypoint.item.obj
                predicted_s = keypoint.item.s - controller.predict_travel_distance_by_t(keypoint.time)
                ret.append(SingleStateProblem(SingleStateSearch(
                    self.base_scenario,
                    MutAgentInfo(
                        self.mut_agent_id,
                        AgentType.NPC,
                        'Sedan',
                        controller,
                        mut_agent_lane,
                        predicted_s - 10,
                        predicted_s + 10
                    ), mut_agent_lane, self.time_limit, self.aut_id
                ), behavior.danger))
        return ret
    
    def _gen_turn_right(self, behavior):
        controllers = self.controller_settings[BehaviorType.TURN_RIGHT] if BehaviorType.TURN_RIGHT in self.controller_settings else self.controller_settings['default']
        ret = []
        for controller in controllers:
            for keypoint in behavior.keypoints:
                mut_agent_lane = keypoint.obj
                predicted_s = keypoint.s - controller.predict_travel_distance_by_t(keypoint.time)
                ret.append(SingleStateProblem(SingleStateSearch(
                    self.base_scenario,
                    MutAgentInfo(
                        self.mut_agent_id,
                        AgentType.NPC,
                        'Sedan',
                        controller,
                        mut_agent_lane,
                        predicted_s - 10,
                        predicted_s + 10
                    ), mut_agent_lane, self.time_limit, self.aut_id
                ), behavior.danger))
        return ret


class HeuristicSearch:
    class Scenario:
        def __init__(self, simulation_result, controller_settings):
            self.base_scenario = simulation_result.scenario
            self.aut_analyzer = simulation_result.aut_analyzer
            self._priority = None
            self._controller_settings = controller_settings
            self.time_limit = simulation_result.scenario.duration
            self.aut_id = simulation_result.scenario.aut_id
            

        @property
        def priority(self):
            if not self._priority is None:
                return self._priority
            self._priority = 0
            for behavior in self.aut_analyzer.behaviors:
                self._priority = max(self._priority, behavior.danger)
            return self._priority

        def __lt__(self, o):
            return self.priority > o.priority

        def __eq__(self, o):
            return self.priority == o.priority

        def __le__(self, o):
            return self.priority >= o.priority

        def generate_problems(self):
            problem_generator = SingleStateSearchGenerator(self.base_scenario, self.aut_analyzer, self._controller_settings, f'npc{len(self.base_scenario.agents)}', self.aut_id, self.time_limit)
            return problem_generator.generate()

    def __init__(self, base_result, max_agent_num,controller_settings):
        self._searched_pattern = set()
        self._unsearched_problems = PriorityQueue()
        self._expandable_scenarios = PriorityQueue()
        # generate new problems from searched scenarios

        self.base_result = base_result 
        self.max_agent_num = max_agent_num
        self.controller_settings = controller_settings

    def search(self):
        if self.max_agent_num <= 0:
            return
        self._expandable_scenarios.put(self.Scenario(self.base_result, self.controller_settings))

        while not self._unsearched_problems.empty() or not self._expandable_scenarios.empty():
            
            unsearched_problem_front = self._unsearched_problems.queue[0] if not self._unsearched_problems.empty() else None
            expandable_scenario_front = self._expandable_scenarios.queue[0] if not self._expandable_scenarios.empty() else None

            if expandable_scenario_front is None or \
                (
                    not unsearched_problem_front is None and \
                    not expandable_scenario_front is None and \
                    unsearched_problem_front.priority >= expandable_scenario_front.priority
                ):
                unsearched_problem_front = self._unsearched_problems.get()
                for result in unsearched_problem_front.search():
                    # print("AUT_PATTERN", aut_analyzer.pattern)
                    yield result
                    if result.collision == True:
                        break
                    if not result.aut_analyzer.pattern in self._searched_pattern:
                        self._searched_pattern.add(result.aut_analyzer.pattern)
                        if len(result.scenario.agents) < len(self.base_result.scenario.agents) + self.max_agent_num:
                            self._expandable_scenarios.put(self.Scenario(result, self.controller_settings))
            else:
                expandable_scenario_front = self._expandable_scenarios.get()
                for problem in expandable_scenario_front.generate_problems():
                    self._unsearched_problems.put(problem)
