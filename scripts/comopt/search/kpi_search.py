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

from comopt.scenario import ScenarioDescription
from comopt.analyzer.analyzer import TraceAnalyzer
from os import times
from comopt.simulator.common import AgentType, CollisionException
from comopt.search import MutAgentInfo, SingleStateSearch
from comopt.analyzer.behavior import *
from queue import PriorityQueue

import math

class KpiDiagnosis(Enum):
    DESIRED = 0
    DISCOMFORT_ACC_DEACC = 2
    INCOMPLETE_ROUTE = 5
    COLLISION = 10
    UNKNOWN_EXCEPTION = -1


class KPIbasedSearch:

    def __init__(self, base_scenario: ScenarioDescription, time_limit: int):


        self.base_scenario = base_scenario
        self.time_limit = time_limit
        # self.time_stamp_granularity 

    def search(self, source_x, source_y, dest_x, dest_y, time_resolution = 0.1):

        # Execute the scenario as 

        kpi = {}
        kpi["undesired"] = False

        try:
            base_trace = self.base_scenario.exec()
            # print(base_traces)
    
            #print(self.is_discomfort_brake_or_acceleration(base_trace, time_resolution))
            #print(self.is_route_completed(base_trace, source_x, source_y, dest_x, dest_y, self.time_limit))

            undesired1, quantity1 = self.is_discomfort_brake_or_acceleration(base_trace, time_resolution)
            if undesired1:
                kpi["undesired"] = True 
                kpi["rationale"] = KpiDiagnosis.DISCOMFORT_ACC_DEACC
        
            undesired2, quantity2 = self.is_route_completed(base_trace, source_x, source_y, dest_x, dest_y, self.time_limit)
            if undesired2:
                kpi["undesired"] = True 
                kpi["rationale"] = KpiDiagnosis.INCOMPLETE_ROUTE


        except CollisionException:
            kpi["undesired"] = True 
            kpi["rationale"] = KpiDiagnosis.COLLISION
        except Exception as e: 
            print(e)
            kpi["undesired"] = True 
            kpi["rationale"] = KpiDiagnosis.UNKNOWN_EXCEPTION
        #key_list = list(base_trace["ego"]._time_record.keys())

        return kpi

        #print(base_trace["ego"]._time_record[key_list[1]].velocity.x)
        #print(base_trace["ego"]._time_record[key_list[1]].velocity.y)



    def is_discomfort_brake_or_acceleration(self, trace, time_stamp_granularity = 0.1):
        # We consider that a brake that can set from 100 km/h (27.78 m/s) to 0 km/h in 4 seconds to be undesirabled. 
        # Similarly, we consider an acceleration to be uncomfortable if something similar in similar amounts
        # This amounts to a quantity of 27.78/4 = 6.94 m/s^2 --> This is the number we use

        # https://www.researchgate.net/post/If_I_got_to_use_a_certain_deceleration_rate_to_identify_unsafe_driving_behaviours_which_could_have_led_to_collision_what_value_seems_to_be_realistic
        # 5.5 m/s^2  - considered as practical maximum
        # 7.5 m/s^2  - considered as uncomfortable

        key_list = list(trace["aut"]._time_record.keys())

        max_acc = 0
        for i in range(1, len(key_list)):
            a_x = (trace["aut"]._time_record[key_list[i]].velocity.x - trace["aut"]._time_record[key_list[i-1]].velocity.x) / time_stamp_granularity
            a_y = (trace["aut"]._time_record[key_list[i]].velocity.y - trace["aut"]._time_record[key_list[i-1]].velocity.y) / time_stamp_granularity

            if (a_x ** 2 + a_y ** 2 ) > 6.94 ** 2:
                return [True, None]
            else:
                max_acc = max (max_acc, math.sqrt(a_x ** 2 + a_y ** 2 ))
                

        return  [False, max_acc]

        None
    
    def is_lateral_jerk(self, trace):
        # See study on lateral jerk 
        # https://link.springer.com/content/pdf/10.1007/s40864-015-0012-y.pdf
        None

    def is_route_completed(self, trace, source_x, source_y, dest_x, dest_y, simulation_time: int, time_stamp_granularity = 0.1):
        key_list = list(trace["aut"]._time_record.keys())

        if(len(key_list) == int(round(simulation_time / time_stamp_granularity)) + 1):
            # Compute ratio of completeness 
            remaining_x = trace["aut"]._time_record[key_list[-1]].position.x - dest_x
            remaining_y = trace["aut"]._time_record[key_list[-1]].position.y - dest_y
            dist_x = dest_x - source_x
            dist_y = dest_y - source_y

            return [False, math.sqrt(remaining_x ** 2 + remaining_y ** 2 ) / math.sqrt(dist_x ** 2 + dist_y ** 2 )]
        else: 
            #print(len(key_list))
            #print((simulation_time / time_stamp_granularity))

            return [True, None]



    def is_near_collision(self, trace):
        None
