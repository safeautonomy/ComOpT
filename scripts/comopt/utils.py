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

from enum import Enum
import inspect, bisect
from typing import Any, Dict, Union
import pickle
from itertools import permutations

from loguru import logger

def Or(iter):
    ret = False
    for i in iter:
        ret = ret or i
    return ret

def And(iter):
    ret = True
    for i in iter:
        ret = ret and i
    return ret

def Match(obj_seq, type_seq):
    assert len(obj_seq) == len(type_seq)
    for objs in permutations(obj_seq):
        for o, t in zip(objs, type_seq):
            if not isinstance(o, t):
                break
        else:
            return tuple(objs)
    return None

def check_type(f):
    def check(*args, **kwargs):
        argspec = inspect.getfullargspec(f)
        names = argspec.args
        annotations = argspec.annotations
        it = zip(args, names)
        for (a, n) in it:
            try:
                annotation = annotations[n]
            except:
                continue
            if annotation.__dict__.get('__origin__') == Union:
                for t in annotation.__args__:
                    if isinstance(a, t):
                        break
                else:
                    raise TypeError(f"Argument '{n}' should have '{annotation}' type, but {type(a)} {a} is given.")       
            elif type(annotation) == type:
                if not isinstance(a, annotation):
                    raise TypeError(f"Argument '{n}' should have '{annotation}' type, but {type(a)} {a} is given.")
        return f(*args, **kwargs)
    return check

class DispatchMethod:
    def __init__(self, func):
        if not callable(func) and not hasattr(func, '__get__'):
            raise TypeError(f'{func!r} is not callable or a descriptor')
        
        self.default_func = func
        self.func_list = []

    def register(self, func):
        self.func_list.append(func)
        return func

    def __get__(self, obj, cls):
        def _method(*args, **kwargs):
            for func in self.func_list:
                argspec = inspect.getfullargspec(func)
                names = argspec.args[1:]
                annotations = argspec.annotations
                defaults = argspec.defaults
                legal = True

                if len(names) < len(args) + len(kwargs):
                    continue
                
                if not And(k in names[len(args):] for k in kwargs.keys()):
                    continue

                if Or(k in names[:len(args)] for k in kwargs.keys()):
                    continue

                if not And(arg in kwargs for arg in names[len(args): None if defaults is None else -len(defaults)]):
                    continue
                
                if not And(isinstance(arg, annotations[name]) for arg, name in zip(args, names[:len(args)]) if name in annotations and type(annotations[name]) == type):
                    continue

                return func(obj, *args, **kwargs)
            
            return self.default_func(obj, *args, **kwargs)
        
        return _method
        
class DictVector:
    def __init__(self, vector):
        self._values = []
        self._paths = []
        for elem in vector:
            assert len(elem) == 2
            assert isinstance(elem[0], tuple)
            self._paths.append(elem[0])
            self._values.append(elem[1])

    def items(self):
        return tuple(zip(self.paths, self.values))
    
    @property
    def values(self):
        return tuple(self._values)

    @property
    def paths(self):
        return tuple(self._paths)

    def __str__(self):
        ret = 'DictVector('
        for item in self.items():
            ret += str(item)
        ret += ')'
        return ret
    
    def __repr__(self):
        return self.__str__()
    
        

class DictObj(dict):
    @DispatchMethod
    def __init__(self, *args, **kwargs):
        super(DictObj, self).__init__(*args, **kwargs)

    @__init__.register
    def _(self, vector:DictVector):
        for path, val in vector.items():
            self.set_item_by_path(path, val)

    def __getattr__(self, key):
        if not key in self.keys():
            self[key] = DictObj()
        if isinstance(self[key], dict):
            return DictObj(self[key])
        return self[key]
    
    def __setattr__(self, key, value):
        self[key] = value

    def to_vec(self):
        def get_values(elem, path):
            values = []
            if isinstance(elem, dict):
                for k, v in elem.items():
                    path.append(k)
                    values.extend(get_values(v, path))
                    path.pop()
            else:
                values.append((tuple(path), elem))
            return values
        vector = DictVector(get_values(self, []))
        return vector
        

    def get_item_by_path(self, path):
        obj = self
        for path_item in path:
            obj = obj[path_item]
        return obj

    def set_item_by_path(self, path, val):
        obj = self
        for path_item in path[:-1]:
            if not path_item in obj:
                obj[path_item] = DictObj()
            if not isinstance(obj[path_item], dict):
                obj[path_item] = DictObj()
            obj = obj[path_item]
        obj[path[-1]] = val

    def to_dict(self):
        ret = DictObj()
        for k, v in self.items():
            ret[k] = v
        return ret

    def copy(self):
        ret = DictObj()
        for k, v in self.items():
            ret[k] = v
        return ret

    def format_str(self, indent = 2):
        def gen_str(elem, depth):
            ret = ''
            if isinstance(elem, dict):
                ret += '{\n'
                cnt = 0
                for k, v in elem.items():
                    cnt += 1
                    ret += ' ' * (depth + 1) * indent + repr(k) + ': ' + gen_str(v, depth + 1).strip()
                    if cnt != len(elem):
                        ret += ',\n'
                if ret[-1] != '\n':
                    ret += '\n'
                ret += ' ' * depth * indent + '}'
                if ret[-1] != '\n':
                    ret += '\n'
                return ret
            elif isinstance(elem, tuple) or isinstance(elem, list) or isinstance(elem, set):
                ret = ''
                origin = repr(elem)
                count = origin.count('\n') 
                cnt = 0
                for ch in origin:
                    ret += ch
                    if ch == '\n':
                        cnt += 1
                        if cnt != count:
                            ret += ' ' * (depth + 1) * indent   
                        else:
                            ret += ' ' * depth * indent
                return ret
            else:
                return repr(elem)
        return gen_str(self, 0).strip()
    
    def __str__(self):
        return self.format_str(2)
    
    def __repr__(self) -> str:
        return self.format_str(2)

def retry(warn_msg="", times=None):
    assert times is None or (isinstance(times, int) and times > 0)
    def executer(func):
        def f(*args, **kwargs):
            cnt = 0
            while times is None or cnt < times:
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    cnt += 1
                    show = f'{warn_msg}: {e}. Retrying ({cnt}' + (f'/{times}' if not times is None else '') + ').'
                    logger.warning(show)
            raise Exception(f"Failed after retry {times} times")
        return f
    return executer

class TimeFunction:
    @DispatchMethod
    def __init__(self):
        self._time_record = dict()

    @__init__.register
    def _(self, config:dict):
        self._time_record = dict()
        for k, v in config.items():
            self._time_record[float[k]] = v

    @property
    def start_time(self):
        return self.ticks[0]

    @property
    def end_time(self):
        return self.ticks[-1]

    @property
    def ticks(self):
        return sorted(list(self._time_record.keys()))

    @property
    def time_step(self):
        ticks = self.ticks
        return ticks[1] - ticks[0]

    def add(self, t, y):
        assert t >= 0
        assert not t in self._time_record.keys()
        self._time_record[t] = y

    def get(self, t):
        ticks = self.ticks
        assert ticks[0] <= t <= ticks[-1]
        
        if t in ticks:
            return self._time_record[t]
        
        left_t = ticks[bisect.bisect_left(ticks, t) - 1]
        right_t = ticks[bisect.bisect_right(ticks, t)]
        v = self._time_record[left_t if t - left_t < right_t - t else right_t]
        return v

    def __contains__(self, t):
        ticks = self.ticks
        return ticks[0] <= t <= ticks[-1]

    def has_record(self, t):
        return t in self._time_record

    def __getitem__(self, t) -> Any:
        if isinstance(t, slice):
            low = t.start
            high = t.stop
            ret = TimeFunction()
            ret[low] = self[low]
            if high != low:
                ret[high] = self[high]
            for t, v in self.iter():
                if low < t < high:
                    ret[t] = v
            return ret
        return self.get(t)

    def __setitem__(self, t, y):
        self._time_record[t] = y

    def iter(self):
        return ((t, self.get(t)) for t in self.ticks)

    @property
    def values(self):
        return tuple(self.get(t) for t in self.ticks)

    def save(self, filename):
        with open(filename, 'wb') as f:
            pickle.dump(self, f)

    def copy(self):
        ret = TimeFunction()
        for t, y in self.iter():
            ret.add(t, y)

    def add_delay(self, t, delay):
        new_time_record = dict()
        for t_, y in self.iter():
            if t_ < t:
                new_time_record[t_] = y
            else:
                new_time_record[t_ + delay] = y
        self._time_record = new_time_record

    def offset(self, offset):
        ret = TimeFunction()
        for t, y in self.iter():
            if t + offset < 0:
                continue
            ret.add(t + offset, y)
        first_tick = ret.ticks[0]
        if first_tick != 0:
            ret.add(0.0, ret[first_tick])
        return ret

    def __repr__(self):
        ret = '['
        for t, v in self.iter():
            ret += f'<{t} : {v}>, '
        ret = ret[:-2]
        ret += ']'
        return ret

    def __str__(self):
        return self.__repr__()

    def to_dict(self):
        ret = dict()
        for t in self.ticks:
            item = self._time_record[t]
            if isinstance(item, Enum):
                ret[t] = item.name
            else:
                ret[t] = item.to_dict()
        return ret
