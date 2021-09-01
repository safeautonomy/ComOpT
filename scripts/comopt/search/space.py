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

from collections import namedtuple
from itertools import permutations
from re import sub
from typing import Dict, Generator, Iterable, Mapping, Type
from comopt.utils import DispatchMethod, DictObj
from enum import Enum
import random
import json

class SpaceBase:
    def iter(self):
        return NotImplementedError

class DiscreteSpace(SpaceBase):
    def __init__(self, value_set):
        self.value_set = set(value_set)
    
    def uniform_sample(self):
        return random.choice(list(self.value_set))

    def iter(self):
        for value in self.value_set:
            yield value

    def __str__(self) -> str:
        return 'DiscreteSpace(' + str(self.value_set) + ')'

    def __repr__(self) -> str:
        return str(self.value_set)
    
class ContinuousSpace(SpaceBase):
    def __init__(self, min_val, max_val):
        assert type(min_val) == type(max_val)
        assert min_val <= max_val
        self.min_val = min_val
        self.max_val = max_val
    
    def uniform_sample(self):
        if type(self.min_val) == int:
            return random.randint(self.min_val, self.max_val)
        elif type(self.min_val) == float:
            return random.uniform(self.min_val, self.max_val)
        else:
            raise TypeError

    def iter(self):
        if isinstance(self.min_val, int):
            for i in range(self.min_val, self.max_val + 1):
                yield i
        else:
            raise TypeError("Continuous float space is not iterable")

    def __str__(self) -> str:
        return f'ContinousSpace([{self.min_val}, {self.max_val}])'

    def __repr__(self) -> str:
        return f'[{self.min_val}, {self.max_val}]'



class NeqSpace(SpaceBase):
    def __init__(self, value_set, dimension) -> None:
        assert 0 < dimension <= len(value_set)
        self.value_set = set(value_set)
        self.dimension = dimension

    def uniform_sample(self):
        result = list(self.value_set)
        random.shuffle(result)
        return tuple(result[:self.dimension])

    def iter(self):
        for seq in permutations(self.value_set, self.dimension):
            yield seq

    def __str__(self) -> str:
        return f'NeqSpace({self.value_set}, dim={self.dimension})'
    
    def __repr__(self) -> str:
        return '\033[34m' + self.__str__() + '\033[0m'


class StructSpace(DictObj, SpaceBase):
    def __init__(self, *args, **kwargs):
        super(StructSpace, self).__init__(*args, **kwargs)

    def _common_sample(self, method):
        def sub_sample(elem):
            if isinstance(elem, dict):
                ret = {}
                for k, v in elem.items():
                    ret[k] = sub_sample(v)
                return DictObj(ret)
            # elif isinstance(elem, list) or isinstance(elem, tuple):
            #     ret = []
            #     for item in elem:
            #         ret.append(sub_sample(item))
            #     return type(elem)(ret)
            elif isinstance(elem, SpaceBase):
                return getattr(elem, method)()
            else:
                return elem            
        return sub_sample(self)


    def iter(self):
        class SeqItem:
            def __init__(self, value, generator, path, space):
                self.value = value; self.generator = generator; self.path = path; self.space = space
        sequence = [SeqItem(next(generator), generator, path, space) for path, space, generator in [(path, space, space.iter()) for path, space in self.to_vec().items() if isinstance(space, SpaceBase)]]

        def gen_next(i):
            try:
                sequence[i].value = next(sequence[i].generator)
                return True
            except StopIteration:
                if i == 0:
                    return False
                sequence[i].generator = sequence[i].space.iter()
                sequence[i].value = next(sequence[i].generator)
                return gen_next(i-1)
        
        last_index = len(sequence) - 1

        def get_ret():
            ret = self.copy()
            for item in sequence:
                ret.set_item_by_path(item.path, item.value)
            return ret
        
        yield get_ret()
        while gen_next(last_index):
            yield get_ret()

    def uniform_sample(self):
        return self._common_sample('uniform_sample')
    
    def __hash__(self):
        return hash(list(self.keys())[0])

class Space(SpaceBase):
    @DispatchMethod
    def __init__(self, *args, sequence=False):
        assert len(args) > 0
        if len(args) > 1:
            sequence = True
        self.spaces = []
        self.sequence = sequence
        for space in args:
            if isinstance(space, SpaceBase):
                self.spaces.append(space)
            elif isinstance(space, set):
                self.spaces.append(DiscreteSpace(space))
            elif isinstance(space, tuple) or isinstance(space, list):
                assert len(space) == 2 
                self.spaces.append(ContinuousSpace(space[0], space[1]))
            else:
                raise TypeError(type(space))
                
    @__init__.register
    def _(self, min_val:int, max_val:int):
        self.spaces = []
        self.sequence = False
        self.spaces.append(ContinuousSpace(min_val, max_val))
    
    @__init__.register
    def _(self, min_val:float, max_val:float):
        self.spaces = []
        self.sequence = False
        self.spaces.append(ContinuousSpace(min_val, max_val))
        
    def _common_sample(self, method):
        result = []
        for space in self.spaces:
            # result.append(space.uniform_sample())
            result.append(getattr(space, method)())
        return tuple(result)
    
    def uniform_sample(self):
        result = self._common_sample('uniform_sample')
        if len(result) == 1 and not self.sequence:
            return result[0]
        else:
            return result

    def iter(self):
        class SeqItem:
            def __init__(self, value, generator):
                self.value = value; self.generator = generator
        sequence = [SeqItem(next(generator), generator) for generator in [space.iter() for space in self.spaces]]
        
        def gen_next(i):
            try:
                sequence[i].value = next(sequence[i].generator)
                return True
            except StopIteration:
                if i == 0:
                    return False
                sequence[i].generator = self.spaces[i].iter()
                sequence[i].value = next(sequence[i].generator)
                return gen_next(i-1)
        
        last_index = len(sequence) - 1
        
        def get_ret():
            return tuple(item.value for item in sequence) if self.sequence else tuple(item.value for item in sequence)[0]

        yield get_ret()
        while gen_next(last_index):
            yield get_ret()
            


    def __str__(self) -> str:
        ret = ''
        if len(self.spaces) == 1:
            ret = 'Space(' + repr(self.spaces[0]) + ')'
        else:
            ret = 'Space('
            for i in range(len(self.spaces)):
                if isinstance(self.spaces[i], StructSpace):
                    ret += ' '.join(repr(self.spaces[i]).split()).replace('\033[0m', '').replace('\033[34m', '')
                else:
                    ret += repr(self.spaces[i])
                if i != len(self.spaces) - 1:
                    ret += ', '
            ret += ')'
        return ret
    
    def __repr__(self) -> str:
        return '\033[34m' + self.__str__() + '\033[0m'
                
