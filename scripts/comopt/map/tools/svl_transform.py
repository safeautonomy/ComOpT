import lgsvl
import argparse, os, sys, json

sys.path.append('../../../')
from comopt.config import LIBDIR

from comopt.geometry import Vector
from comopt.map import MapParser
from comopt.map.tools.converter import Converter
from comopt.map.config import map_config
from comopt.map.transform import SVLTransform
from comopt.utils import check_type

import logging

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description="Generate STD Map based on SVL Simulator.")
    argparser.add_argument('map_name', help='map_name listed in the map config.')
    argparser.add_argument('--sim_addr', '-a', nargs=1, help='Address of the SVL Simulator host.', default='localhost', metavar='sim_addr')
    argparser.add_argument('--sim_port', '-p', nargs=1, help='Port of the SVL Simulator host.', default=8181, metavar='sim_port')
    args = argparser.parse_args()

    try:
        sim = lgsvl.Simulator(address=args.sim_addr, port=args.sim_port)
    except:
        logging.error('Can not connect to the SVL Simulator.')
        exit(-1)

    transformer = SVLTransform(sim, args.map_name)
    converter = Converter(transformer)

    with open(map_config.base_map_path(args.map_name), 'rb') as f:
        gps_map = MapParser()
        gps_map.parse_from_bin(f.read())
    
    std_map = converter.transform(gps_map)

    with open(os.path.join(map_config.folder(args.map_name), 'svl_map.bin'), 'wb') as f:
        f.write(std_map.dump())
    
    