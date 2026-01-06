# sumoITScontrol/control/ramp_metering/ALINEA.py

import numpy as np

class ALINEA:
    def __init__(self, params, ramp_meter):
        self.params = params
        self.ramp_meter = ramp_meter
        # init measurement data
        self.measurement_data = {}
        for sensor in self.ramp_meter.mainline_sensors:
            self.measurement_data[sensor] = {}
            self.measurement_data[sensor]["occupancy_l_interval"] = [[0, 0]]
        self.measurement_data["metering_rate"] = [[0, 100.0]]
        self.measurement_data["metering_occupancy"] = [[0, 0.0]]
        self.measurement_data["queue_length_m"] = [[0, 0]]
        self.measurement_data["previous_metering_rate"] = 100.0
        self.measurement_data["previous_occupancy"] = 0.0
        self.measurement_data["counter"] = -1
        self.measurement_data["time"] = 0
    
    def execute_control(self, current_time):
        """
        This function implements the ALINEA ramp metering controller (which is updating the metering rates).
        This function also conducts necessary measurements of the merging area.
        """
        self.measurement_data["counter"] += 1
        if self.measurement_data["counter"] == self.params["measurement_period"]:
            self.measurement_data["counter"] = 0
            # determine ramp states
            mainline_state = self.ramp_meter._get_mainline_state("getLastIntervalOccupancy")
            queue_state = self.ramp_meter._get_queue_state("getLastIntervalMaxJamLengthInMeters")
            current_occupancy = np.nanmean(mainline_state)       
            current_queue_length = np.nanmean(queue_state)
            # alinea law
            alinea_metering_rate = self.measurement_data["previous_metering_rate"] + self.params["K_P"] * (self.params["target_occupancy"] - current_occupancy) + self.params["K_I"] * (current_occupancy - self.measurement_data["previous_occupancy"])
            alinea_metering_rate = min(max(alinea_metering_rate, self.params["min_rate"]), self.params["max_rate"])
            # record
            for idx, sensor in enumerate(self.ramp_meter.mainline_sensors):
                self.measurement_data[sensor]["occupancy_l_interval"].append([current_time, mainline_state[idx]])        
            self.measurement_data["metering_rate"].append([current_time, alinea_metering_rate])
            self.measurement_data["metering_occupancy"].append([current_time, current_occupancy])
            self.measurement_data["previous_occupancy"] = current_occupancy
            self.measurement_data["previous_metering_rate"] = alinea_metering_rate
            self.measurement_data["queue_length_m"].append([current_time, current_queue_length])
            # control
            self.ramp_meter._update_ramp_signal_control_logic_alinea(
                metering_rate=alinea_metering_rate/100, 
                cycle_duration=self.params["cycle_duration"]
            )