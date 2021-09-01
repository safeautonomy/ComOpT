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

from comopt.simulator import SimulationResult
from comopt.search import HeuristicSearch
from comopt.analyzer.behavior import BehaviorType
from comopt.controller import LaneFollowController 
from comopt.scenario import ScenarioDescription
from comopt.abstract_scenario import abstract_scenario_management
from comopt.abstract_scenario import abstract_scenario_gen
from comopt.concrete_scenario import scenario_instantiation
from comopt.config import LIBDIR
from loguru import logger
import comopt.map_parse.map_parse as mp
import argparse, os, json, random
import pandas as pd
import argparse

parser = argparse.ArgumentParser(prog="AD Tester")

parser.add_argument('OUTPUT_DIR', default='./ad_tester_output', help="the folder of generated scenarios")
parser.add_argument('-en', '--equiv_num', type=int, default=15, help='specify the number of equivalence classes to consider')
parser.add_argument('-sn', '--sample_num', type=int, default=3, help='specify the number of samples of each equivalence class')
parser.add_argument('-mp', '--max_perturb_iteration', type=int, default=20, help='specify the maximum number of perturb iteration')


args = parser.parse_args()

MAP_NAME = 'SanFrancisco'
MAP_FOLDER_NAME = 'san_francisco'

OUTPUT_DIR = args.OUTPUT_DIR


ABSTRACT_SCENARIO_DIR = os.path.join(OUTPUT_DIR, 'abstract_scenario')
CONCRETE_SCENARIO_DIR = os.path.join(OUTPUT_DIR, 'concrete_scenario')
BASE_RESULT_DIR = os.path.join(OUTPUT_DIR, 'base_result')
PERTURBATION_RESULT_DIR = os.path.join(OUTPUT_DIR, 'perturb_result')

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(ABSTRACT_SCENARIO_DIR, exist_ok=True)
os.makedirs(CONCRETE_SCENARIO_DIR, exist_ok=True)
os.makedirs(BASE_RESULT_DIR, exist_ok=True)
os.makedirs(PERTURBATION_RESULT_DIR, exist_ok=True)

META_MODEL_FILE = os.path.join(LIBDIR, '../data/meta_model/SF_meta_model.xml')
DOMAIN_RESTRICTIONS_FILE = os.path.join(LIBDIR, '../data/meta_model/SF_domain_restrictions.xml')
# ABSTRACT_SCENARIO_OUTPUT_FOLDER = "competition_output_scenarios/abstract_scenarios/"
ABSTRACT_SCENARIO_OUTPUT_FOLDER = os.path.join(OUTPUT_DIR, 'abstract_scenario_xml')
os.makedirs(ABSTRACT_SCENARIO_OUTPUT_FOLDER, exist_ok=True)
EQUIV_CLASS_NUM = args.equiv_num
SAMPLE_NUM = args.sample_num
MAX_PERTURBATION_ITERATION = args.max_perturb_iteration

random.seed(6)
mp.load_map(MAP_FOLDER_NAME)

abstract_scenarios = []

try:
    for equiv_class_id in range(EQUIV_CLASS_NUM):
        with open(os.path.join(ABSTRACT_SCENARIO_DIR, f'abstract_scenario_{equiv_class_id}.json')) as f:
            abstract_scenario = json.load(f)
        abstract_scenarios.append(abstract_scenario)
    logger.info("Abstract scenarios found.")
except Exception as e:
    gen_abstract = input('Abstract scenaiors are not exist or incomplete. Generate abstract scenarios (y/n): ')
    if gen_abstract == 'y':
        abstract_scenarios = []
        abstract_scenario_metric_manager = abstract_scenario_management.ScenarioKProjectionMetricManager(META_MODEL_FILE, k_value=2)
        abstract_scenario_metric_manager.add_domain_restrictions_from_file(DOMAIN_RESTRICTIONS_FILE)
        variable_assignment = abstract_scenario_gen.propose_scenario_candidate(abstract_scenario_metric_manager, False)
        for equiv_class_id in range(EQUIV_CLASS_NUM):
            variable_assignment = abstract_scenario_gen.propose_scenario_candidate(abstract_scenario_metric_manager, False)
            abstract_scenario_output_path = os.path.join(ABSTRACT_SCENARIO_OUTPUT_FOLDER, "abst_sce_"+str(equiv_class_id)+".xml")
            abstract_scenario_metric_manager.write_scenario_to_file(variable_assignment, abstract_scenario_output_path)
            abstract_scenario_metric_manager.add_scenarios_from_file(abstract_scenario_output_path)
            abstract_scenario = abstract_scenario_metric_manager.translate_assignment_to_abstract_scenario(variable_assignment)
            with open(os.path.join(ABSTRACT_SCENARIO_DIR, f'abstract_scenario_{equiv_class_id}.json'), 'w') as f:
                json.dump(abstract_scenario, f)
            abstract_scenarios.append(abstract_scenario)
    else:
        exit()
abstract_scenario_summary = pd.DataFrame(abstract_scenarios)
abstract_scenario_summary.to_csv(os.path.join(OUTPUT_DIR, 'abstract_scenario_summary.csv'))



concrete_scenarios = []
try:
    for equiv_class_id in range(EQUIV_CLASS_NUM):
        concrete_scenarios.append([])
        for sample_id in range(SAMPLE_NUM):
            with open(os.path.join(CONCRETE_SCENARIO_DIR, f'concrete_scenario_{equiv_class_id}_{sample_id}.json')) as f:
                concrete_scenarios[equiv_class_id].append(ScenarioDescription(json.load(f)))
    logger.info("Concrete scenarios found.")


except:
    gen_scenario_description = input("Concrete scenarios are not exist or incomplete. Generate concrete scenario descriptoins (y/n): ")
    if gen_scenario_description == 'y':
        concrete_scenarios = []
        concrete_scenario_instantiator = scenario_instantiation.ScenarioInstantiator(
            os.path.join(LIBDIR, "../data/meta_model/SF_parameter_range.json"), 
            os.path.join(LIBDIR, f"../output/{MAP_FOLDER_NAME}/junction_classify.bin"), 
            MAP_NAME
        )
        for equiv_class_id in range(EQUIV_CLASS_NUM):
            concrete_scenarios.append([])
            abstract_scenario = abstract_scenarios[equiv_class_id]
            for sample_id in range(SAMPLE_NUM):
                logger.info(f"Generating concrete scenario equiv_class_id={equiv_class_id}, sample_id={sample_id}")
                scenario_runner = concrete_scenario_instantiator.sample_from_abstract_scenario(abstract_scenario)
                scenario_runner.save(os.path.join(CONCRETE_SCENARIO_DIR, f'concrete_scenario_{equiv_class_id}_{sample_id}.json'))
                concrete_scenarios[equiv_class_id].append(scenario_runner)
    else:
        exit()

exceptions = []

for equiv_class_id in range(EQUIV_CLASS_NUM):
    for sample_id in range(SAMPLE_NUM):
        logger.info(f"Running scenario equiv_class_id={equiv_class_id}, sample_id={sample_id}")
        scenario_runner = ScenarioDescription(os.path.join(CONCRETE_SCENARIO_DIR, f'concrete_scenario_{equiv_class_id}_{sample_id}.json'))
        result = scenario_runner.exec()
        result.save(os.path.join(BASE_RESULT_DIR, f'scenario_{equiv_class_id}_{sample_id}_result.json'), report=True)
        exception = result.report['exception']
        if len(exception) > 0:
            exceptions.append({
                'file': os.path.join('base_result', f'scenario_{equiv_class_id}_{sample_id}_result.json'),
                'exception(s)': exception
            })
            logger.warning(f'exception found while running scenario equiv_class_id={equiv_class_id}, sample_id={sample_id}: {exception}')


with open(os.path.join(LIBDIR, '../data/meta_model/SF_parameter_range.json')) as f:
    parameter_range = json.load(f)
for equiv_class_id in range(EQUIV_CLASS_NUM):
    abstract_scenario = abstract_scenarios[equiv_class_id]
    for sample_id in range(SAMPLE_NUM):
        with open(os.path.join(BASE_RESULT_DIR, f'scenario_{equiv_class_id}_{sample_id}_result.json')) as f:
            data = json.load(f)
            base_result = SimulationResult(data)
        perturb_save_dir = os.path.join(PERTURBATION_RESULT_DIR, f'scenario_{equiv_class_id}_{sample_id}_perturb')
        os.makedirs(perturb_save_dir, exist_ok=True)
        
        npc_num = 0
        for agent in base_result.scenario.agents.values():
            if agent.agent_type.name == 'NPC':
                npc_num += 1
        search_problem = HeuristicSearch(base_result, parameter_range["density_of_vehicles"][abstract_scenario["density_of_vehicles"]][1] - npc_num, {
            BehaviorType.FOLLOW_LANE: [LaneFollowController(max_speed = 0), LaneFollowController(max_speed = 5)],
            'default': [LaneFollowController(max_speed = 5)]
        })

        try:
            if MAX_PERTURBATION_ITERATION == 0:
                break
            cnt = 0    
            logger.info(f"Running perturbation for scenario equiv_class_id={equiv_class_id}, sample_id={sample_id}")
            logger.info(f"Start iteration {cnt+1}")
            for result in search_problem.search():
                if cnt >= MAX_PERTURBATION_ITERATION:
                    break
                perturb_save_file_name = f'scenario_{equiv_class_id}_{sample_id}_perturb_{cnt}_result.json'
                result.save(os.path.join(perturb_save_dir, perturb_save_file_name))
                exception = result.report['exception']
                if len(exception) > 0:
                    exceptions.append({
                        'file': os.path.join('perturb_result', f'scenario_{equiv_class_id}_{sample_id}_perturb_{cnt}_result.json'),
                        'exception(s)': exception
                    })
                    logger.warning(f'exception found while running perturbation on scenario equiv_class_id={equiv_class_id}, sample_id={sample_id}, iter={cnt}: {exception}')
                cnt += 1
                logger.info(f"Start iteration {cnt+1}")
        except Exception as e:
            pass

exceptions_pd = pd.DataFrame(exceptions)
exceptions_pd.to_csv(os.path.join(OUTPUT_DIR, 'problem.csv'))


