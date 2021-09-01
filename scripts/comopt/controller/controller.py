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

from comopt.geometry import Vector, lerp
from comopt.utils import DictObj, DispatchMethod, TimeFunction
import json, pickle
from websocket import create_connection

class ControllerBase:
    name = 'controller_base'
    def __init__(self, params):
        pass

    def close(self):
        pass

class LaneFollowController(ControllerBase):
    name = 'lane_follow_controller'

    @DispatchMethod
    def __init__(self, max_speed):
        self.max_speed = max_speed

    @__init__.register
    def _(self, config:dict):
        self.max_speed = config['max_speed']

    def to_dict(self):
        ret = dict()
        ret['controller_name'] = self.name
        ret['parameters'] = {}
        ret['parameters']['max_speed'] = self.max_speed
        return ret

    def predict_travel_distance_by_t(self, travel_time):
        travel_distance = 0
        t = 0
        cur_speed = 0
        TIMEDELTA = 0.1
        while t < travel_time:
            if cur_speed <= self.max_speed - 1e-5:
                if t <= 5:
                    speed_adjust_rate = lerp(1, 4, t / 5)
                else:
                    speed_adjust_rate = 4
                cur_speed += speed_adjust_rate * TIMEDELTA * (self.max_speed - cur_speed)
            travel_distance += cur_speed * TIMEDELTA
            t += TIMEDELTA       
        return travel_distance

class ApolloController(ControllerBase):
    name = 'apollo_controller'
    @DispatchMethod
    def __init__(
            self, 
            destination, 
            address:str='localhost', 
            enable_modules=('Localization', 'Transform', 'Routing', 'Prediction', 'Planning', 'Control'), 
            dreamview_port:int=8888, 
            bridge_port:int=9090
        ):

        self.destination = Vector(destination.x, destination.y)

        self.address = address
        self.dreamview_port = dreamview_port
        self.bridge_port = bridge_port
        self.enable_modules = enable_modules
        
        self._ws = None

    @__init__.register
    def _(self, config:dict):
        self.destination = Vector(config['destination'])
        self.address = config['address']
        self.dreamview_port = config['dreamview_port']
        self.bridge_port = config['bridge_port']
        self.enable_modules = config['enable_modules']

        self._ws = None

    @property
    def ws(self):
        if self._ws is None:
            self._ws = create_connection('ws://' + self.address + ':' + str(self.dreamview_port) + '/websocket')
        return self._ws

    def to_dict(self):
        ret = dict()
        ret['controller_name'] = self.name
        ret['parameters'] = {}
        ret['parameters']['destination'] = self.destination.to_dict()
        ret['parameters']['address'] = self.address
        ret['parameters']['dreamview_port'] = self.dreamview_port
        ret['parameters']['bridge_port'] = self.bridge_port
        ret['parameters']['enable_modules'] =  list(self.enable_modules)
        return ret

    def send_routing_request(self, start_gps, heading, destination):
        data = json.dumps({
            "end": {
                "x": destination.x,
                "y": destination.y,
                "z": 0
            },
            "start": {
                "x": start_gps.x,
                "y": start_gps.y,
                "z": 0,
                "heading": heading
            },
            "type": "SendRoutingRequest",
            "waypoint": []
        })
        self.ws.send(data)

    def start_modules(self, modules):
        for module in modules:
            d = {
                'action': 'START_MODULE',
                'type': 'HMIAction',
                'value': module
            }
            self.ws.send(json.dumps(d))
        # flag = True
        # while flag:
        #     flag = False
        #     d = json.loads(self.ws.recv())
        #     if d['type'] == 'HMIStatus':
        #         for module in modules:
        #             if d['data']['modules'][module] == False:
        #                 flag = True
        #                 break

    def stop_modules(self, modules):
        for module in modules:
            d = {
                'action': 'STOP_MODULE',
                'type': 'HMIAction',
                'value': module
            }
            self.ws.send(json.dumps(d))
        # flag = True
        # while flag:
        #     flag = False
        #     d = json.loads(self.ws.recv())
        #     if d['type'] == 'HMIStatus':
        #         for module in modules:
        #             if d['data']['modules'][module] == True:
        #                 flag = True
        #                 break

    def close(self):
        if not self._ws is None:
            self.ws.close()
        pass

class StaticController(ControllerBase):
    name = 'static_controller'

    def __init__(self):
        self.destination = Vector()
        pass

class SingleDestinationController(ControllerBase):
    name = 'single_destination_controller'
    
    @DispatchMethod
    def __init__(self, destination, speed, trigger_distance):
        self.destination = destination
        self.speed = speed
        self.trigger_distance = trigger_distance
    
    @__init__.register
    def _(self, config:dict):
        self.destination = Vector(config["destination"])
        self.speed = config["speed"]
        self.trigger_distance = config["trigger_distance"]

    def to_dict(self):
        ret = dict()
        ret['controller_name'] = self.name
        ret['parameters'] = {}
        ret['parameters']['destination'] = self.destination.to_dict()
        ret['parameters']['speed'] = self.speed
        ret['parameters']['trigger_distance'] = self.trigger_distance
        return ret

class RecordController(ControllerBase):
    name = 'record_controller'

    @DispatchMethod
    def __init__(self, trace, scale = 1, delay = 0, stop_distance=0, min_speed=0.1):
        self._trace_file = None
        if isinstance(trace, str):
            self._trace_file = trace
            with open(trace, 'rb') as f:
                trace = pickle.load(f)
        else:
            trace = trace

        ticks = trace.ticks
        assert 0 in ticks

        self._trace = trace

        self._scale = scale
        self._delay = delay
        self._stop_distance = stop_distance
        assert min_speed > 0
        self._min_speed = min_speed

    @__init__.register
    def _(self, config: dict):
        if isinstance(config['trace'], str):
            with open('trace', 'rb') as f:
                self._trace = pickle.load(f)
        else:
            self._trace = TimeFunction(config['trace'])
        
        self._scale = config['scale'] if 'scale' in config else 1
        self._delay = config['delay'] if 'delay' in config else 0
        self._stop_distance = config['stop_distance'] if 'stop_distance' in config else 0
        self._min_speed = config['min_speed'] if 'min_speed' in config else 0.1

    def to_dict(self):
        ret = dict()
        ret['controller_name'] = self.name
        ret['parameters'] = {}
        ret['parameters']['trace'] = self._trace_file if not self._trace_file is None else self.trace.to_dict()
        ret['parameters']['scale'] = self.scale
        ret['parameters']['delay'] = self.delay
        ret['parameters']['stop_distance'] = self.stop_distance
        ret['parameters']['min_speed'] = self.min_speed
        return ret

    @property
    def min_speed(self):
        return self._min_speed

    @property
    def stop_distance(self):
        return self._stop_distance

    @property
    def trace(self):
        return self._trace

    @property
    def scale(self):
        return self._scale

    @property
    def delay(self):
        return self._delay
            

    
class ControllerFactory:
    @classmethod
    def get_controller(cls, config):
        controller_name = config['controller_name']
        if controller_name == 'lane_follow_controller':
            return LaneFollowController(config['parameters'])
        elif controller_name == 'apollo_controller':
            return ApolloController(config['parameters'])
        elif controller_name == 'record_controller':
            return RecordController(config['parameters'])
        elif controller_name == 'single_destination_controller':
            return SingleDestinationController(config['parameters'])
        else:
            raise ValueError('Controller', controller_name, 'not found')
