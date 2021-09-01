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

import lgsvl
import os, math, json
from lgsvl.dreamview import CoordType
from collections import Iterable
import time, logging
from numpy import isin

log = logging.getLogger(__name__)

class Connection(lgsvl.dreamview.Connection):
    def __init__(self, simulator, ego_agent, ip=os.environ.get("LGSVL__AUTOPILOT_0_HOST", "localhost"), port="8888"):
        super(Connection, self).__init__(simulator, ego_agent, ip, str(port))

    def set_destination(self, long_east, lat_north):
        if not isinstance(long_east, Iterable):
            long_east = [long_east]
        if not isinstance(lat_north, Iterable):
            lat_north = [lat_north]

        assert len(long_east) == len(lat_north)

        current_pos = self.ego.state.transform
        current_gps = self.sim.map_to_gps(current_pos)
        heading = math.radians(current_gps.orientation)

        # Start position should be the position of the GPS
        # Unity returns the position of the center of the vehicle so adjustment is required
        northing_adjustment = (
            math.sin(heading) * self.gps_offset.z - math.cos(heading) * self.gps_offset.x
        )
        easting_adjustment = (
            math.cos(heading) * self.gps_offset.z + math.sin(heading) * self.gps_offset.x
        )

        gps_waypoints = []
        for x_long_east, z_lat_north in zip(long_east, lat_north):
            transform = lgsvl.Transform(
                lgsvl.Vector(x_long_east, 0, z_lat_north), lgsvl.Vector(0, 0, 0)
            )
            gps = self.sim.map_to_gps(transform)
            dest_x = gps.easting
            dest_y = gps.northing
            gps_waypoints.append({"x": dest_x, "y": dest_y, "z": 0})

        self.ws.send(
            json.dumps(
                {
                    "type": "SendRoutingRequest",
                    "start": {
                        "x": current_gps.easting + easting_adjustment - math.cos(heading) * 2,
                        "y": current_gps.northing + northing_adjustment - math.sin(heading) * 2,
                        "z": 0,
                        "heading": heading,
                    },
                    "end": gps_waypoints[-1],
                    "waypoint": gps_waypoints[:-1],
                }
            )
        )

        return
    
    def enable_apollo(self, dest_x, dest_z, modules, load_time=5):
        """
        Enables a list of modules and then sets the destination
        """
        for mod in modules:
            log.info("Starting {} module...".format(mod))
            self.enable_module(mod)

        time.sleep(load_time)
        self.set_destination(dest_x, dest_z)
