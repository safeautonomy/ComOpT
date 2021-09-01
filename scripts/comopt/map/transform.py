import lgsvl
import comopt.simulator as simulator
from comopt.geometry import Vector
from comopt.utils import check_type
from comopt.map import map_config

class SVLTransform:
    def __init__(self, sim:lgsvl.Simulator, map_name=None):
        self.sim = sim
        if not map_name is None:
            self.load_map(map_name)
    
    # @check_type
    def std_from_sim(self, sim_vec:lgsvl.Vector):
        return Vector(sim_vec.z, -sim_vec.x, sim_vec.y)

    # @check_type
    def std_to_sim(self, std_vec:Vector):
        return lgsvl.Vector(-std_vec.y, std_vec.z, std_vec.x)

    def load_map(self, map_name):
        if self.sim.current_scene == map_config.sim_name(simulator.SimulatorType.SVL, map_name):
            return
        svl_name = map_config.sim_name(simulator.SimulatorType.SVL, map_name)
        self.sim.load(svl_name)


    # @check_type
    def std_to_gps(self, std_pos:Vector):
        state = lgsvl.AgentState()
        position = self.std_to_sim(std_pos)
        state.transform.position.x, state.transform.position.y, state.transform.position.z = position.x, position.y, position.z
        gps_info = self.sim.map_to_gps(state.transform)
        return Vector(gps_info.easting, gps_info.northing)

    def std_from_gps(self, gps_pos):
        sim_state = self.sim.map_from_gps(easting=gps_pos.x, northing=gps_pos.y)
        sim_state.position.y = 100
        hit = self.sim.raycast(sim_state.position, lgsvl.Vector(0, -1, 0), 1)
        if hit is None:
            sim_state.position.y = 0
            return self.std_from_sim(sim_state.position)
        else:
            return self.std_from_sim(hit.point)