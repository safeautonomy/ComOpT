#!/usr/bin/env python3

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


# Fix the random seed (6 for the team size) for repeatability
import random
random.seed(6)
import json, os, sys


from comopt import scenario
from comopt.scenario import ScenarioDescription
from loguru import RecordProcess, logger

import argparse

parser = argparse.ArgumentParser(prog="Scenario Runner")

parser.add_argument('scenario_file', help="JSON file name of scenario description")
parser.add_argument('--save', '-s', nargs=1, metavar='file_path', help="save the execution result to file_path")

args = parser.parse_args()

# args.scenario_file = 'scenarios/scenario_9_description_0.json'
# args.save = ['test_result/scenario_9_result_0.json']

OUTPUT_DIR = './ad_tester_output'

ABSTRACT_SCENARIO_DIR = os.path.join(OUTPUT_DIR, 'abstract_scenario')
CONCRETE_SCENARIO_DIR = os.path.join(OUTPUT_DIR, 'concrete_scenario')

META_MODEL_FILE = 'data/meta_model/SF_meta_model.xml'
DOMAIN_RESTRICTIONS_FILE = 'data/meta_model/SF_domain_restrictions.xml'
ABSTRACT_SCENARIO_OUTPUT_FOLDER = "competition_output_scenarios/abstract_scenarios/"

logger.info(f"Running {args.scenario_file}")
scenario = ScenarioDescription(args.scenario_file)
if args.save is None:
    scenario.time_step = None
result = scenario.exec()
# print(args.save)
if not args.save is None:
    result.save(args.save[0], report=True)
    logger.info(f"Result saved to {args.save[0]}")
    exit()
else:
    logger.info(f"Running finished")
    exit()


