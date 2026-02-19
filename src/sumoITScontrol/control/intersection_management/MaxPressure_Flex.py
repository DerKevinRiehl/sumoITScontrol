# sumoITScontrol/control/intersection_management/MaxPressure_Flex.py

import random


class MaxPressure_Flex:
    def __init__(self, params, intersection):
        self.params = params
        self.intersection = intersection
        # init measurement data
        self.measurement_data = {}
        self.measurement_data["counter"] = -1
        self.measurement_data["time"] = 0
        self.measurement_data["fsm_timer"] = -1
        self.measurement_data["current_gt_start"] = 0
        self.measurement_data["current_signal_phase"] = 0
        self.measurement_data["next_signal_phase"] = -1
        self.measurement_data["current_fsm_state"] = "start"
        self.measurement_data["pressures"] = []
        self.measurement_data["history_green_phase"] = []

    def execute_control(self, current_time):
        """
        This function implements the MaxPressure (Flexibel) ramp metering controller (which is updating the metering rates).
        This function also conducts necessary measurements of the surrounding intersection area.
        """
        self.measurement_data["counter"] += 1
        if self.measurement_data["counter"] == self.params["measurement_period"]:
            self.measurement_data["counter"] = 0
            self.measurement_data["fsm_timer"] += 1
            self.measurement_data["pressures"] = (
                self.intersection.get_queue_lengths_num_vehicles()
            )
            if self.measurement_data["current_fsm_state"] == "start":
                if self.measurement_data["fsm_timer"] == self.params["G_T_MIN"]:
                    self.measurement_data["current_fsm_state"] = "check_pressures"
                    self.measurement_data["fsm_timer"] = -1
            elif self.measurement_data["current_fsm_state"] == "check_pressures":
                # pressure index corresponds to green-phase index (phase/2)
                idx = int(self.measurement_data["current_signal_phase"] / 2)
                current_pressure = (
                    self.measurement_data["pressures"][idx]
                    if idx < len(self.measurement_data["pressures"])
                    else 0
                )
                other_pressures = (
                    max(self.measurement_data["pressures"])
                    if len(self.measurement_data["pressures"]) > 0
                    else 0
                )
                if current_pressure < other_pressures:
                    self.measurement_data["current_fsm_state"] = "next_phase"
                    self.measurement_data["fsm_timer"] = -1
                else:
                    self.measurement_data["current_fsm_state"] = "wait"
                    self.measurement_data["fsm_timer"] = -1
            elif self.measurement_data["current_fsm_state"] == "wait":
                if self.measurement_data["fsm_timer"] == self.params["T_A"]:
                    current_gt = (
                        current_time - self.measurement_data["current_gt_start"]
                    )
                    if current_gt > self.params["G_T_MAX"]:
                        self.measurement_data["current_fsm_state"] = "next_phase"
                        self.measurement_data["fsm_timer"] = -1
                    else:
                        self.measurement_data["current_fsm_state"] = "check_pressures"
                        self.measurement_data["fsm_timer"] = -1
            elif self.measurement_data["current_fsm_state"] == "next_phase":
                # pick the highest pressure among other approaches
                valid_indices = [
                    i
                    for i in range(len(self.measurement_data["pressures"]))
                    if i != int(self.measurement_data["current_signal_phase"] / 2)
                ]
                if not valid_indices:
                    # fallback: stay in same phase
                    self.measurement_data["next_signal_phase"] = self.measurement_data[
                        "current_signal_phase"
                    ]
                else:
                    max_pressure = max(
                        self.measurement_data["pressures"][i] for i in valid_indices
                    )
                    max_indices = [
                        i
                        for i in valid_indices
                        if self.measurement_data["pressures"][i] == max_pressure
                    ]
                    chosen = random.choice(max_indices)
                    self.measurement_data["next_signal_phase"] = int(chosen * 2)
                # begin transition (set yellow)
                self.measurement_data["current_signal_phase"] += 1
                self.measurement_data["fsm_timer"] = -1
                self.measurement_data["current_fsm_state"] = "transition"
            elif self.measurement_data["current_fsm_state"] == "transition":
                if self.measurement_data["fsm_timer"] == self.params["T_L"]:
                    # switch to selected phase
                    self.measurement_data["current_signal_phase"] = (
                        self.measurement_data["next_signal_phase"]
                    )
                    self.measurement_data["history_green_phase"].append([current_time, self.measurement_data["next_signal_phase"]])
                    self.measurement_data["next_signal_phase"] = -1
                    self.measurement_data["fsm_timer"] = -1
                    self.measurement_data["current_fsm_state"] = "start"
                    self.measurement_data["current_gt_start"] = current_time
            else:
                print("WARNING UNKNOWN STATE")
            # apply the current phase to the traffic lights
            self.intersection.set_signal_on_traffic_lights(
                phase=int(self.measurement_data["current_signal_phase"])
            )
