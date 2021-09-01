from math import log
import sys, os

from comopt.map.parser import MapParser
from comopt.utils import check_type
from loguru import logger


class Converter:
    def __init__(self, transformer):
        self.transformer = transformer
    
    # @check_type
    def transform(self, gps_map:MapParser):
        std_map = gps_map.copy()
        self._transform_lanes(std_map)
        return std_map
    
    def _transform_lanes(self, std_map):
        for lane in std_map.lane:
            logger.info(f'transform {lane.id.id}')
            for point in lane.left_boundary.curve.segment[0].line_segment.point:
                std_pos = self.transformer.std_from_gps(point)
                point.x = std_pos.x
                point.y = std_pos.y

            for point in lane.right_boundary.curve.segment[0].line_segment.point:
                std_pos = self.transformer.std_from_gps(point)
                point.x = std_pos.x
                point.y = std_pos.y

            for point in lane.central_curve.segment[0].line_segment.point:
                std_pos = self.transformer.std_from_gps(point)
                point.x = std_pos.x
                point.y = std_pos.y
            
        for signal in std_map.map.signal:
            logger.info(f'transform {signal.id.id}')
            for point in signal.boundary.point:
                std_pos = self.transformer.std_from_gps(point)
                point.x = std_pos.x
                point.y = std_pos.y
                point.z = point.z - std_pos.z
            
            for point in signal.stop_line[0].segment[0].line_segment.point:
                std_pos = self.transformer.std_from_gps(point)
                point.x = std_pos.x
                point.y = std_pos.y

        for crosswalk in std_map.map.crosswalk:
            logger.info(f'transform {crosswalk.id.id}')
            for point in crosswalk.polygon.point:
                std_pos = self.transformer.std_from_gps(point)
                point.x = std_pos.x
                point.y = std_pos.y
