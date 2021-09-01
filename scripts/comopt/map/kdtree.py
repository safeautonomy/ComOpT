from enum import Enum
from collections import namedtuple
import math

math_epsilon = 1e-6

class SegmentInfo:
    def __init__(self, segment, lane, s):
        self.segment = segment
        self.lane = lane
        self.s = s
    
    def max_x(self):
        return max(self.segment.p1.x, self.segment.p2.x)

    def min_x(self):
        return min(self.segment.p1.x, self.segment.p2.x)

    def max_y(self):
        return max(self.segment.p1.y, self.segment.p2.y)

    def min_y(self):
        return min(self.segment.p1.y, self.segment.p2.y)

class KDTreeNode:
    class Partition(Enum):
        X = 1
        Y = 2

    def __init__(self, objects):
        self._objects_sorted_by_min = []
        self._objects_sorted_by_max = []
        self._objects_sorted_by_min_bound = []
        self._objects_sorted_by_max_bound = []
        self._num_objects = 0
        self._boundary = self._compute_boundary(objects)
        
        self._mid_x = (self._boundary.max_x + self._boundary.min_x) / 2
        self._mid_y = (self._boundary.max_y + self._boundary.min_y) / 2
        
        self._partition, self._partition_position = self._compute_partition()

        self._left_node = None
        self._right_node = None

        left_node_objects, right_node_objects, other_objects = self._partition_objects(objects)
        
        if len(left_node_objects) > 0:
            self._left_node = KDTreeNode(left_node_objects)

        if len(right_node_objects) > 0:
            self._right_node = KDTreeNode(right_node_objects)

        self._init_objects(other_objects)

        for i in range(self._num_objects - 1):
            assert self._objects_sorted_by_max_bound[i] >= self._objects_sorted_by_max_bound[i+1]
            assert self._objects_sorted_by_min_bound[i] <= self._objects_sorted_by_min_bound[i+1]


    def _compute_boundary(self, objects):
        BoundaryInfo = namedtuple('BoundaryInfo', ['max_x', 'min_x', 'max_y', 'min_y'])
        result = BoundaryInfo(-float('inf'), float('inf'), -float('inf'), float('inf'))
        for object in objects:
            result = BoundaryInfo(
                max_x = max(result.max_x, object.max_x()),
                min_x = min(result.min_x, object.min_x()),
                max_y = max(result.max_y, object.max_y()),
                min_y = min(result.min_y, object.min_y())
            )
        return result

    def _compute_partition(self):
        if self._boundary.max_x - self._boundary.min_x >= self._boundary.max_y - self._boundary.min_y:
            return self.Partition.X, self._mid_x
        else:
            return self.Partition.Y, self._mid_y

    def _init_objects(self, objects):
        self._num_objects = len(objects)
        self._objects_sorted_by_min = [item for item in objects]
        self._objects_sorted_by_max = [item for item in objects]

        self._objects_sorted_by_min.sort(key=lambda segment_info: segment_info.min_x() if self._partition == self.Partition.X else segment_info.min_y())
        self._objects_sorted_by_max.sort(key=lambda segment_info: -segment_info.max_x() if self._partition == self.Partition.X else -segment_info.max_y())

        self._objects_sorted_by_min_bound = [(object.min_x() if self._partition == self.Partition.X else object.min_y()) for object in self._objects_sorted_by_min]
        self._objects_sorted_by_max_bound = [(object.max_x() if self._partition == self.Partition.X else object.max_y()) for object in self._objects_sorted_by_max]
        
        for i in range(self._num_objects - 1):
            assert self._objects_sorted_by_max_bound[i] >= self._objects_sorted_by_max_bound[i+1]
            assert self._objects_sorted_by_min_bound[i] <= self._objects_sorted_by_min_bound[i+1]

    def _partition_objects(self, objects):
        left_node_objects = []
        right_node_objects = []
        other_objects = []
        if self._partition == self.Partition.X:
            for object in objects:
                if object.max_x() <= self._partition_position:
                    left_node_objects.append(object)
                elif object.min_x() >= self._partition_position:
                    right_node_objects.append(object)
                else:
                    other_objects.append(object)
        else:
            for object in objects:
                if object.max_y() <= self._partition_position:
                    left_node_objects.append(object)
                elif object.min_y() >= self._partition_position:
                    right_node_objects.append(object)
                else:
                    other_objects.append(object)
        return left_node_objects, right_node_objects, other_objects

    def _lower_distance_to_point(self, point):
        dx = 0
        if point.x < self._boundary.min_x:
            dx = self._boundary.min_x - point.x
        elif point.x > self._boundary.max_x:
            dx = point.x - self._boundary.max_x

        dy = 0
        if point.y < self._boundary.min_y:
            dy = self._boundary.min_y - point.y
        elif point.y > self._boundary.max_y:
            dy = point.y - self._boundary.max_y

        return math.sqrt(dx * dx + dy * dy)

    def get_all_objects(self):
        ret = []
        if not self._left_node is None:
            ret += self._left_node.get_all_objects()

        ret += self._objects_sorted_by_min

        if not self._right_node is None:
            ret += self._right_node.get_all_objects()
        return ret

    def get_closest_object(self, point, cur_closest_object=None, cur_min_distance=float('inf')):
        if self._lower_distance_to_point(point) >= cur_min_distance - math_epsilon:     
            return cur_closest_object, cur_min_distance
        
        pvalue = point.x if self._partition == self.Partition.X else point.y

        search_left_first = True if pvalue < self._partition_position else False

        if search_left_first:
            if not self._left_node is None:
                cur_closest_object, cur_min_distance = self._left_node.get_closest_object(point, cur_closest_object, cur_min_distance)
        else:
            if not self._right_node is None:
                cur_closest_object, cur_min_distance = self._right_node.get_closest_object(point, cur_closest_object, cur_min_distance)

        if cur_min_distance <= math_epsilon:
            return cur_closest_object, cur_min_distance

        if search_left_first:
            for i in range(self._num_objects):
                bound = self._objects_sorted_by_min_bound[i]
                if bound > pvalue and bound - pvalue > cur_min_distance:
                    break
                object = self._objects_sorted_by_min[i]
                distance = object.segment.dis_to_point(point).distance
                if distance < cur_min_distance:
                    cur_min_distance = distance
                    cur_closest_object = object
        else:
            for i in range(self._num_objects):
                bound = self._objects_sorted_by_max_bound[i]
                if bound < pvalue and pvalue - bound > cur_min_distance:
                    break
                object = self._objects_sorted_by_max[i]
                distance = object.segment.dis_to_point(point).distance
                if distance < cur_min_distance:
                    cur_min_distance = distance
                    cur_closest_object = object
        
        if cur_min_distance <= math_epsilon:
            return cur_closest_object, cur_min_distance

        if search_left_first:
            if not self._right_node is None:
                return self._right_node.get_closest_object(point, cur_closest_object, cur_min_distance)
        else:
            if not self._left_node is None:
                return self._left_node.get_closest_object(point, cur_closest_object, cur_min_distance)
        
        return cur_closest_object, cur_min_distance


class KDTree:
    def __init__(self, objects):
        self._root = KDTreeNode(objects)

    def get_closet_object(self, point):
        return self._root.get_closest_object(point)[0]