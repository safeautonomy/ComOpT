import sys, os
from copy import deepcopy

from comopt.config import LIBDIR
sys.path.append(os.path.join(LIBDIR, 'proto_lib/'))
import comopt.proto_lib.modules.map.proto.map_pb2 as map_pb2
from google.protobuf import text_format

class MapParser:
    def __init__(self):
        self.map = map_pb2.Map()

    def parse_from_bin(self, binary_proto):
        self.map.ParseFromString(binary_proto)

    def parse_from_text(self, text_proto):
        self.map = text_format(text_proto, map_pb2.Map())

    def dump(self):
        return self.map.SerializeToString()

    def dumps(self):
        return str(self.map)

    def copy(self):
        map = MapParser()
        map.parse_from_bin(self.dump())
        return map

    def __getattr__(self, name):
        return getattr(self.map, name)

    def __setattr__(self, name, value):
        if name == 'map':
            super().__setattr__(name, value)
        else:
            self.map.__setattr__(name, value)