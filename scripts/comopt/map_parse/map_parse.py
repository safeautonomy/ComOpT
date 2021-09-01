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

import json

import comopt.map_parse as mp
from comopt import map

from comopt.map_parse.abstract_junction import abstract_junction, two_tier_cluster
from comopt.map_parse.crosswalk import parse_crosswalks, add_crosswalks_info_in_junctions, init_crosswalk_map
from comopt.map_parse.junction import parse_junction, parse_sections_as_junctions
from comopt.map_parse.junction_classes import JCrossroads, JYShaped, JTShaped, try_junction_classes, \
    JunctionClass, J2ends, J2EndsStraight, J2EndsLong, J2EndsLongStraight, J3ends, J4ends
from comopt.map_parse.map_formatter import MapFormatter
# from comopt.simulator import Simulator
from comopt.map_parse.signal import init_signal_map, add_signals_info_in_junctions, parse_signals
from comopt.simulator import SimulatorType
import comopt.database as db
import pickle
from loguru import logger as log
import os
from comopt.map_parse import global_map

map_name_file_to_sim = {
    'san_francisco': 'SanFrancisco',
    'borregas_ave': 'BorregasAve',
    'gomentum': 'GoMentum',
    'gomentum_dlt': 'GoMentumDlt',
    'cubetown': 'Cubetowm',
    'autonomous_stuff': 'AutonomousStuff',
    'shalun': 'Shalun',
    'single_lane_road': 'SingleLaneRoad',
    'straight_1_lane_pedestrian_crosswalk': 'Straight1LanePedestrianCrosswalk',
    'wide_flat_map': 'WideFlatMap'
}


def check_file_exists(map_name, obj_name):
    return os.path.exists(f'output/{map_name}/{obj_name}.bin')


def save_to_file(map_name, obj_name, obj):
    if not os.path.exists(f'output/{map_name}'):
        os.mkdir(f'output/{map_name}')
    with open(f'output/{map_name}/{obj_name}.bin', 'wb') as f:
        pickle.dump(obj, f)
    log.debug(f'Stored: {map_name}/{obj_name}.bin')


def save_as_text(map_name, obj_name, obj):
    if not os.path.exists(f'output/{map_name}'):
        os.mkdir(f'output/{map_name}')
    with open(f'output/{map_name}/{obj_name}.txt', 'w') as f:
        f.write(str(obj))
    log.debug(f'Stored: {map_name}/{obj_name}.txt')


def save_as_json(map_name, obj_name, obj):
    if not os.path.exists(f'output/{map_name}'):
        os.mkdir(f'output/{map_name}')
    with open(f'output/{map_name}/{obj_name}.json', 'w') as f:
        f.write(json.dumps(obj, indent=4))
    log.debug(f'Stored: {map_name}/{obj_name}.json')


def read_from_file(map_name, obj_name):
    with open(f'output/{map_name}/{obj_name}.bin', 'rb') as f:
        ret = pickle.load(f)
    log.debug(f'Loaded: {map_name}/{obj_name}.bin')
    return ret


def load_map(name):
    log.debug('Start loading map')
    if name not in map_name_file_to_sim.keys():
        log.error(f'Unrecorded map name "{name}"')
        exit()
    m = map.Map(SimulatorType.SVL, map_name_file_to_sim[name])
    mp.global_map = m
    log.debug('Finish loading map')
    return m


def map_format(m):
    log.debug('Start map formatting...')
    mf = MapFormatter(m)
    mf.format_vertices()
    mf.format_edges()
    log.debug('Finish map formatting')
    return mf.g


def import_to_db(g):
    log.debug('Start importing to db...')
    c = db.Converter()
    c.open_session()
    c.clear()
    log.debug('clear finish')
    c.add_graph(g)
    c.close_session()
    c.close()
    log.debug('Finish importing to db.')


def get_junctions():
    log.debug('Start getting junctions...')
    junction_list = []
    i = 0
    while True:
        junction = parse_junction(f"J_{i}")
        if len(junction.connectors) == 0:
            # log.debug(f'search done: J_{i}')
            break
        junction_list.append(junction)
        if i % 20 == 0:
            log.info(f'get J_{i} ({i} / unknown): {junction.info()}')
        i += 1
    junction_list.sort()
    log.debug(f'Finished getting junctions, got {len(junction_list)} junctions')
    return junction_list


def get_sections_as_junctions():
    log.debug('Start getting sections as junctions...')
    js = parse_sections_as_junctions()
    log.debug('Finish getting sections as junctions.')
    return js


def sort_lanes_in_connectors(jl):
    log.debug('Start sorting lanes in connectors...')
    sel = db.Selector()
    sel.open_session()
    cnt = 0
    for j in jl:
        for k in j.connectors.keys():
            j.connectors[k].sort_endpoints(sel)
        if cnt % 40 == 0:
            log.info(f'finish sorting lanes in connectors({cnt}/{len(jl)}): {j.id}')
        cnt += 1
    sel.close_session()
    sel.close()
    log.debug('Finish sorting lanes in connectors.')


def add_crosswalks_in_junctions(name, jl):
    load_map(name)
    init_crosswalk_map()
    return add_crosswalks_info_in_junctions(jl, parse_crosswalks())


def add_signals_in_junctions(name, jl):
    load_map(name)
    init_signal_map()
    return add_signals_info_in_junctions(jl, parse_signals())


def abstract_junctions(junction_list):
    log.debug('Start junctions abstraction...')
    abj_list = []
    for j in junction_list:
        try:
            abj = abstract_junction(j)
            # print(f'OK: {j.id}, abj={abj.section_info()}')
            abj_list.append(abj)
            # print(abj.permutation(0))
        except Exception:
            log.error(f'Exception: {j.id}')
    log.debug('Finish junctions abstraction.')
    return abj_list


def junction_cluster(abj_list):
    log.debug('Start junctions cluster...')
    ret = two_tier_cluster(abj_list)
    log.debug('Finish junctions cluster.')
    return ret


def junction_classify(junction_list):
    log.debug('Start junctions classify...')
    junction_class_map = dict()
    for j in junction_list:
        jc = None
        class_name = None
        try:
            jc = try_junction_classes(j, [
                JCrossroads(),
                JYShaped(),
                JTShaped(),
                J2EndsLongStraight(),
                J2EndsLong(),
                J2EndsStraight(),
                J3ends(),
                J4ends(),
                J2ends(),
                JunctionClass()
            ])
            class_name = jc.name()
        except Exception as e:
            log.error(e)
        if class_name is None:
            continue
        if class_name not in junction_class_map.keys():
            junction_class_map[class_name] = list()
        junction_class_map[class_name].append(jc)
    log.debug('Finish junctions classify.')
    return junction_class_map


def parse_junctions(name, update=False):
    if check_file_exists(name, 'junctions') and not update:
        jl = read_from_file(name, 'junctions')
    else:
        m = load_map(name)
        g = map_format(m)
        import_to_db(g)
        jl = get_junctions()
        sort_lanes_in_connectors(jl)
        jl += get_sections_as_junctions()
        jl = add_crosswalks_in_junctions(name, jl)
        jl = add_signals_in_junctions(name, jl)
        save_to_file(name, 'junctions', jl)
    return jl


def parse_junction_cluster(name, update=False, junctions_update=False):
    log.debug('Start parsing junctions cluster')
    jl = parse_junctions(name, junctions_update)
    if check_file_exists(name, 'junction_cluster') and not update:
        jc = read_from_file(name, 'junction_cluster')
    else:
        abj_list = abstract_junctions(jl)
        jc = junction_cluster(abj_list)
        save_to_file(name, 'junction_cluster', jc)

    with open(f'output/{name}/junction_cluster.txt', 'w') as f:
        f.write(str(jc).replace("], '", "],\n'"))


def parse_junction_classify(name, update=False, junctions_update=False):
    log.debug('Start parsing junctions classify')
    if check_file_exists(name, 'junction_classify') and not update:
        jcm = read_from_file(name, 'junction_classify')
    else:
        load_map(name)
        jl = parse_junctions(name, junctions_update)
        jcm = junction_classify(jl)
        save_to_file(name, 'junction_classify', jcm)
        jcm_lanes = {}
        for k in jcm.keys():
            jcm_lanes[k] = []
            for i in range(0, len(jcm[k])):
                jcm_lanes[k].append(jcm[k][i].info())
        save_as_json(name, 'junction_classify', jcm_lanes)
    return jcm


def map_parse(name, update=False):
    parse_junctions(name, update)
    parse_junction_classify(name, update, False)
    parse_junction_cluster(name, update, False)
