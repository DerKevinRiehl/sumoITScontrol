"""sumoITScontrol: Traffic Controller Collection for SUMO Traffic Simulations [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

import numpy as np
import traci
from .simulation_tools import SimulationTools


class RampMeter:
    def __init__(
        self,
        tl_id,
        queue_sensors,
        mainline_sensors,
        smoothening_factor=0.1,
        saturation_flow_veh_per_sec=0.5,
    ):
        self.tl_id = tl_id
        self.queue_sensors = queue_sensors
        self.mainline_sensors = mainline_sensors
        self.identified_sensor_types = False
        self.smoothening_factor = smoothening_factor
        self.saturation_flow_veh_per_sec = saturation_flow_veh_per_sec
        self.flow_smoothed = 0

    def _identify_sensor_types(self):
        self.queue_sensor_types = {}
        self.mainline_sensor_types = {}
        for sensor in self.queue_sensors:
            self.queue_sensor_types[sensor] = SimulationTools.get_sensor_type(sensor)
        for sensor in self.mainline_sensors:
            self.mainline_sensor_types[sensor] = SimulationTools.get_sensor_type(sensor)
        self.identified_sensor_types = True

    def _check_lists_prepared(self):
        if not SimulationTools.sensor_list_initialized:
            SimulationTools.init_sensor_lists(traci)
        if not self.identified_sensor_types:
            self._identify_sensor_types()

    def _get_mainline_state(self, state_func):
        self._check_lists_prepared()
        measurement_data = []
        for sensor in self.mainline_sensors:
            if self.mainline_sensor_types[sensor] == "inductionloop":
                if state_func == "getLastIntervalOccupancy":
                    measurement_data.append(
                        traci.inductionloop.getLastIntervalOccupancy(sensor)
                    )
            else:
                if state_func == "getLastIntervalOccupancy":
                    measurement_data.append(
                        traci.lanearea.getLastIntervalOccupancy(sensor)
                    )
        return measurement_data

    def _get_queue_state(self, state_func):
        self._check_lists_prepared()
        measurement_data = []
        for sensor in self.queue_sensors:
            if self.queue_sensor_types[sensor] == "inductionloop":
                if state_func == "getLastIntervalMaxJamLengthInMeters":
                    measurement_data.append(
                        traci.inductionloop.getLastIntervalMaxJamLengthInMeters(sensor)
                    )
                if state_func == "getLastIntervalVehicleNumber":
                    measurement_data.append(
                        traci.inductionloop.getLastIntervalVehicleNumber(sensor)
                    )
            else:
                if state_func == "getLastIntervalMaxJamLengthInMeters":
                    measurement_data.append(
                        traci.lanearea.getLastIntervalMaxJamLengthInMeters(sensor)
                    )
                if state_func == "getLastIntervalVehicleNumber":
                    measurement_data.append(
                        traci.lanearea.getLastIntervalVehicleNumber(sensor)
                    )
        return measurement_data

    def _update_ramp_signal_control_logic_alinea(
        self, metering_rate: float, cycle_duration: int
    ):
        """
        This function adjusts the traffic light phases of an intersection, based on the metering rate (green time share).

        Parameters:
        - metering_rate: proportion of green time in the cycle (0.0 - 1.0)
        - cycle_duration: total duration of one cycle
        """
        self.current_metering_rate = metering_rate
        # determine red and green time
        green_time = int(metering_rate * cycle_duration)
        red_time = cycle_duration - green_time
        # write new times to traffic light control logic
        traffic_light_logic = traci.trafficlight.getAllProgramLogics(self.tl_id)[0]
        for ph_id in range(0, len(traffic_light_logic.phases)):
            if ph_id == 0:  # G
                traffic_light_logic.phases[ph_id].minDur = green_time
                traffic_light_logic.phases[ph_id].maxDur = green_time
                traffic_light_logic.phases[ph_id].duration = green_time
            elif ph_id == 1:  # R
                traffic_light_logic.phases[ph_id].minDur = red_time
                traffic_light_logic.phases[ph_id].maxDur = red_time
                traffic_light_logic.phases[ph_id].duration = red_time
        traci.trafficlight.setProgramLogic(self.tl_id, traffic_light_logic)
        # reset traffic controller to use new logic
        traci.trafficlight.setPhase(self.tl_id, 0)

    def get_smoothened_flow(self):
        flow = np.nanmean(self._get_queue_state("getLastIntervalVehicleNumber"))
        self.flow_smoothed = (
            self.smoothening_factor * flow
            + (1 - self.smoothening_factor) * self.flow_smoothed
        )
        return self.flow_smoothed
