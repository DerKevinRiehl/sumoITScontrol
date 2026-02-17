import traci
from .simulation_tools import SimulationTools

class Intersection:
    def __init__(self, tl_id, phases, links=None, sensors=None, green_states=None, yellow_states=None):
        self.tl_id = tl_id
        self.phases = phases
        self.links = links
        self.sensors = sensors
        self.green_states = green_states
        self.yellow_states = yellow_states
        if self.sensors is None:
            self.pressure_source = "lanes"
        else:
            self.pressure_source = "sensors"
        
    def set_signal_on_traffic_lights(self, phase):
        """
        This function sets trafficlight to specific phase.
        
        Parameters:
        - phase: the movement phase (int)
        """
        traci.trafficlight.setPhase(self.tl_id, phase)

    def get_queue_lengths_num_vehicles(self):
        pressures = []
        # links is expected to be a mapping phase_index -> list-of-lanes
        n_vehicles = {}
        for phase in self.phases:
            pressure = 0
            if self.pressure_source=="lanes":
                lanes = self.links[phase]
                # count vehicles on lanes
                for lane in lanes:
                    if lane in n_vehicles:
                        continue
                    else:
                        n_vehicles[lane] = traci.lane.getLastStepVehicleNumber(lane)
                # count "hidden" vehicles on internal syntehtic lanes (intersections)
                SimulationTools.determine_hidden_vehicles(traci)
                edges = [l.split("_")[0] for l in lanes]
                hidden_vehicles_current_edge = [element for element in SimulationTools.hidden_vehicles_current_edge if element in edges]
                # determine pressure
                for lane in lanes:
                    pressure += n_vehicles[lane]
                # add pressure from "hidden" vehicles (currently on synthetic lanes of interscetions)
                pressure += len(hidden_vehicles_current_edge)
            else:
                sensors = self.sensors[phase]
                # count vehicles on links
                for sensor in sensors:
                    if sensor in n_vehicles:
                        continue
                    else:
                        n_vehicles[sensor] = traci.lanearea.getLastStepVehicleNumber(sensor)
                # determine pressure
                for sensor in sensors:
                    pressure += n_vehicles[sensor]
            pressures.append(pressure)
        return pressures
    