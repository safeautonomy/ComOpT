
import os, sys, json
from comopt.config import LIBDIR
# from comopt.simulator.common import SimulatorType
# import comopt.simulator as simulator
from comopt.utils import check_type

class MapConfiguration:
    def __init__(self, config_file):
        with open(config_file) as f:
            self.map_config = json.load(f)
    
    @property
    def maps(self):
        return set(self.map_config.keys())

    def info(self, map_name):
        assert map_name in self.maps, f'{map_name} does not exist.'
        return self.map_config[map_name]

    def folder(self, map_name):
        info = self.info(map_name)
        assert 'folder' in info, f'folder is not specifed in the map config'
        return os.path.join(LIBDIR, info['folder'])

    def apollo_name(self, map_name):
        info = self.info(map_name)
        assert 'apollo_name' in info, f'{map_name} is not supported by Apollo'
        return info['apollo_name']

    @check_type
    def sim_name(self, sim_type, map_name:str):
        info = self.info(map_name)
        name_index = sim_type.name.lower() + '_name'
        assert name_index in info, f'{map_name} is not supported by {sim_type.name} Simulator.'
        return info[name_index]

    def base_map_path(self, map_name):
        assert os.path.exists(os.path.join(self.folder(map_name), 'base_map.bin')), f'base_map.bin for {map_name} was not found.'
        return os.path.join(self.folder(map_name), 'base_map.bin')

    @check_type
    def sim_map_path(self, sim_type, map_name):
        map_file = sim_type.name.lower() + '_map.bin'
        assert os.path.exists(os.path.join(self.folder(map_name), map_file)), f'{map_name} is not supported by {sim_type.name} Simulator.'
        return os.path.join(self.folder(map_name), map_file)

map_config = MapConfiguration(os.path.join(LIBDIR, 'config/maps.json'))