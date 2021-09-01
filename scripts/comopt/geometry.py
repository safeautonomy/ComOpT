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
import math
import random
from os import close
import re

from comopt.utils import DictObj, DispatchMethod, check_type
from typing import Dict, Mapping, Tuple, Union


def lerp(min_val, max_val, rate):
    return min_val + (max_val - min_val) * rate


def clamp(val, min_val, max_val):
    return min(max_val, (max(min_val, val)))


class SLPoint:
    def __init__(self, s: Union[float, int] = 0, l: Union[float, int] = 0, obj=None):
        self.s = s
        self.l = l
        self.obj = obj

    @property
    def d(self):
        return abs(self.l)

    def __repr__(self):
        return f'SLPoint({self.s}, {self.l}, {self.obj})'

    def __str__(self):
        return self.__repr__()

    def __eq__(self, sl_point):
        return self.obj == sl_point.obj and self.s == sl_point.s and self.l == sl_point.l


class Vector:
    @DispatchMethod
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    @__init__.register
    def _(self, config: dict):
        self.x = config['x']
        self.y = config['y']
        self.z = 0

    def to_dict(self):
        return {'x': self.x, 'y': self.y}

    def add_x(self, x):
        return Vector(self.x + x, self.y, self.z)

    def add_y(self, y):
        return Vector(self.x, self.y + y, self.z)

    def add_z(self, z):
        return Vector(self.x, self.y, self.z + z)

    def copy(self):
        return Vector(self.x, self.y, self.z)

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def magnitude_3d(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    @property
    def unit(self):
        return self / self.magnitude()

    def rotate(self, r):
        return Vector(math.cos(r) * self.x - math.sin(r) * self.y, math.sin(r) * self.x + math.cos(r) * self.y, self.z)

    @property
    def left_unit(self):
        return self.rotate(math.pi / 2).unit

    @property
    def angle(self):
        unit = self.unit
        if unit.y >= 0:
            return Radian(math.acos(unit.x))
        else:
            return Radian(2 * math.pi - math.acos(unit.x))

    def angle_with(self, vec):
        return math.acos(self.dot(vec) / self.magnitude() / vec.magnitude())

    def angle_with_in_xoy(self, vec):
        if self.x * vec.y - self.y * vec.x > 0:
            return self.angle_with(vec)
        else:
            return - self.angle_with(vec)

    def __add__(self, vec):
        return Vector(self.x + vec.x, self.y + vec.y, self.z + vec.z)

    def __neg__(self):
        return Vector(-self.x, -self.y, -self.z)

    def __sub__(self, vec):
        return Vector(self.x - vec.x, self.y - vec.y, self.z - vec.z)

    def __mul__(self, k):
        return Vector(k * self.x, k * self.y, k * self.z)

    def __rmul__(self, k):
        return Vector(k * self.x, k * self.y, k * self.z)

    def __truediv__(self, k):
        return Vector(self.x / k, self.y / k, self.z / k)

    def dot(self, vec):
        return self.x * vec.x + self.y * vec.y + self.z * vec.z

    # @check_type
    def dis_to(self, vec):
        return (self - vec).magnitude()

    def dis_to_3d(self, vec):
        return (self - vec).magnitude_3d()

    def __str__(self):
        return '({}, {})'.format(self.x, self.y)

    def __repr__(self):
        return 'Vector' + self.__str__()

    def str_3d(self):
        return f'({self.x}, {self.y}, {self.z})'

    def parse_str_3d(self, s):
        f = r'[-+]?(\d+(\.\d*)?|\.\d+)'
        if not re.match(r'\(' + f + r'\,\ ?' + f + r'\,\ ?' + f + r'\)', s):
            raise NameError('Invalid vector string 3d')
        found = re.finditer(f, s)
        nums = []
        for ff in found:
            nums.append(float(s[ff.span()[0]:ff.span()[1]]))
        self.x = nums[0]
        self.y = nums[1]
        self.z = nums[2]
        return self


class Segment:
    @DispatchMethod
    def __init__(self, p1: Vector, p2: Vector):
        self.p1: Vector = p1
        self.p2: Vector = p2

    @__init__.register
    def _(self, config: dict):
        self.p1 = Vector(config['p1'])
        self.p2 = Vector(config['p2'])

    def to_dict(self):
        return {'p1': self.p1.to_dict(), 'p2': self.p2.to_dict()}

    @property
    def length(self):
        return self.p1.dis_to(self.p2)

    @property
    def vec(self):
        return Vector(self.p2.x - self.p1.x, self.p2.y - self.p1.y, self.p2.z - self.p1.z)

    @property
    def unit(self):
        return self.vec.unit

    # @check_type
    def lerp(self, t):
        return self.p1 + self.vec * t

    # @check_type
    def to_left_test(self, point: Vector):
        return self.p1.x * self.p2.y - self.p1.y * self.p2.x + \
               self.p2.x * point.y - self.p2.y * point.x + \
               point.x * self.p1.y - point.y * self.p1.x > 0

    # @check_type
    def cover_parallelly(self, point):
        if Segment(self.p1, self.p1 + self.vec.left_unit).to_left_test(point) ^ \
                Segment(self.p2, self.p2 + self.vec.left_unit).to_left_test(point):
            return True
        else:
            return False

    # @check_type
    def dis_to_point(self, point: Vector):
        DisInfo = namedtuple('DisInfo', ['distance', 'point'])
        cross = (self.p2.x - self.p1.x) * (point.x - self.p1.x) + (self.p2.y - self.p1.y) * (point.y - self.p1.y)
        if cross <= 0:
            result = DisInfo(point.dis_to(self.p1), Vector(self.p1.x, self.p1.y))
            return result
        d2 = (self.p2.x - self.p1.x) ** 2 + (self.p2.y - self.p1.y) ** 2
        if cross >= d2:
            result = DisInfo(point.dis_to(self.p2), Vector(self.p2.x, self.p2.y))
            return result

        r = cross / d2
        closest_point = Vector(
            lerp(self.p1.x, self.p2.x, r),
            lerp(self.p1.y, self.p2.y, r)
        )
        result = DisInfo(point.dis_to(closest_point), closest_point)
        return result

    def get_sl_by_xy(self, point: Vector):
        if self.cover_parallelly(point):
            dis_info = self.dis_to_point(point)
            s = dis_info.point.dis_to(self.p1)
            l = dis_info.distance
            if not self.to_left_test(point):
                l = -l
            return SLPoint(s, l, self)
        else:
            return None

    def get_random_point(self):
        return self.p1 + (self.p1 - self.p2) * random.random()

    def __str__(self):
        return 'Segment({}, {})'.format(str(self.p1), str(self.p2))

    def __repr__(self):
        return 'Segment({}, {})'.format(self.p1.__repr__(), self.p2.__repr__())


class Radian:
    @DispatchMethod
    def __init__(self, r=0):
        self.r = (r + (-r // (2 * math.pi) + 1) * 2 * math.pi) % (2 * math.pi)

    def to_dict(self):
        return self.r

    def rotate(self, r):
        self.r = (self.r + r + (-r // (2 * math.pi) + 1) * 2 * math.pi) % (2 * math.pi)

    @property
    def unit(self):
        return Vector(math.cos(self.r), math.sin(self.r))

    @property
    def angle(self):
        return self.r / math.pi * 180

    def __add__(self, r):
        if isinstance(r, Radian):
            return Radian(self.r + r.r)
        elif isinstance(r, int) or isinstance(r, float):
            return Radian(self.r + r)
        else:
            raise TypeError()

    def __sub__(self, r):
        return self.__add__(-r)

    def __neg__(self):
        return Radian(-self.r)

    def __repr__(self):
        return f'{self.r} rad'

    def __str__(self):
        return f'{self.r} rad'


def check_intersect(seg1, seg2):
    p1 = (seg1.p1.x, seg1.p1.y)
    p2 = (seg1.p2.x, seg1.p2.y)
    p3 = (seg2.p1.x, seg2.p1.y)
    p4 = (seg2.p2.x, seg2.p2.y)

    def cross(p1, p2, p3):
        x1 = p2[0] - p1[0]
        y1 = p2[1] - p1[1]
        x2 = p3[0] - p1[0]
        y2 = p3[1] - p1[1]
        return x1 * y2 - x2 * y1

    if (max(p1[0], p2[0]) >= min(p3[0], p4[0]) \
            and max(p3[0], p4[0]) >= min(p1[0], p2[0]) \
            and max(p1[1], p2[1]) >= min(p3[1], p4[1]) \
            and max(p3[1], p4[1]) >= min(p1[1], p2[1])):
        if (cross(p1, p2, p3) * cross(p1, p2, p4) <= 0
                and cross(p3, p4, p1) * cross(p3, p4, p2) <= 0):
            D = True
        else:
            D = False
    else:
        D = False
    return D


def intersect_point(seg1, seg2):
    if not check_intersect(seg1, seg2):
        return None
    point_is_exist = False
    x = y = 0
    x1, y1, x2, y2 = seg1.p1.x, seg1.p1.y, seg1.p2.x, seg1.p2.y
    x3, y3, x4, y4 = seg2.p1.x, seg2.p1.y, seg2.p2.x, seg2.p2.y

    if (x2 - x1) == 0:
        k1 = None
        b1 = 0
    else:
        k1 = (y2 - y1) * 1.0 / (x2 - x1)
        b1 = y1 * 1.0 - x1 * k1 * 1.0
    if (x4 - x3) == 0:
        k2 = None
        b2 = 0
    else:
        k2 = (y4 - y3) * 1.0 / (x4 - x3)
        b2 = y3 * 1.0 - x3 * k2 * 1.0

    if k1 is None:
        if not k2 is None:
            x = x1
            y = k2 * x1 + b2
            point_is_exist = True
    elif k2 is None:
        x = x3
        y = k1 * x3 + b1
    elif not k2 == k1:
        x = (b2 - b1) * 1.0 / (k1 - k2)
        y = k1 * x * 1.0 + b1 * 1.0
        point_is_exist = True

    if point_is_exist:
        return Vector(x, y)
    else:
        return None
