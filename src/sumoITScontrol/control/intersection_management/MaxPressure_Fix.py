# sumoITScontrol/control/intersection_management/MaxPressure_Fix.py

import numpy as np

class MaxPressure_Fix:
    def __init__(self, params, intersection):
        self.params = params
        self.intersection = intersection
        # init measurement data
        self.measurement_data = {}
        self.measurement_data["counter"] = -1
        self.measurement_data["time"] = 0
        self.measurement_data["fsm_timer"] = -1
        self.measurement_data["current_signal_phase"] = 0
        self.measurement_data["next_signal_phase"] = -1
        self.measurement_data["current_fsm_state"] = "idle"  # idle | green | transition
        self.measurement_data["pressures"] = []
        self.measurement_data["pressures_hist"] = []
        self.measurement_data["schedule"] = (
            []
        )  # green durations (seconds) for each phase in order
        self.measurement_data["schedule_index"] = 0
        self.measurement_data["current_gt_start"] = 0
        self.measurement_data["history_schedule"] = []

    def _compute_schedule_from_pressures(self):
        n = len(self.intersection.phases)
        if n == 0:
            return []

        # Step 1: compute average pressures
        self.measurement_data["pressures"] = (
            np.mean(np.asarray(self.measurement_data["pressures_hist"]), axis=0).tolist()
            if self.measurement_data.get("pressures_hist")
            else [0] * n
        )
        self.measurement_data["pressures_hist"] = []
    
        # Step 2: effective green time after losses
        total_loss = n * self.params["T_L"]
        effective_green = max(0.0, float(self.params["cycle_duration"]) - float(total_loss))
    
        pressures = self.measurement_data["pressures"]
        total_pressure = sum(pressures) if pressures else 0
    
        # Step 3: initial allocation proportional to pressure
        if total_pressure <= 0:
            greens = [effective_green / n] * n
        else:
            greens = [p / total_pressure * effective_green for p in pressures]
    
        # Step 4: enforce min/max constraints
        greens = [max(self.params["G_T_MIN"], min(g, self.params["G_T_MAX"])) for g in greens]
    
        # Step 5: redistribute leftover/deficit
        total_alloc = sum(greens)
        diff = effective_green - total_alloc
    
        # Iteratively adjust to exactly match effective_green
        tolerance = 1e-6
        while abs(diff) > tolerance:
            if diff > 0:
                adjustable = [i for i, g in enumerate(greens) if g < self.params["G_T_MAX"]]
            else:
                adjustable = [i for i, g in enumerate(greens) if g > self.params["G_T_MIN"]]
            if not adjustable:
                break
            adjust_amount = diff / len(adjustable)
            for i in adjustable:
                greens[i] += adjust_amount
                greens[i] = max(self.params["G_T_MIN"], min(greens[i], self.params["G_T_MAX"]))
            total_alloc = sum(greens)
            diff = effective_green - total_alloc
    
        # Step 6: round to nearest integer
        greens = [int(round(g)) for g in greens]
    
        # Step 7: final tiny adjustment to ensure exact sum
        delta = int(round(effective_green)) - sum(greens)
        for i in range(abs(delta)):
            index = i % n
            if delta > 0:
                if greens[index] < self.params["G_T_MAX"]:
                    greens[index] += 1
            elif delta < 0:
                if greens[index] > self.params["G_T_MIN"]:
                    greens[index] -= 1
    
        return greens

    def execute_control(self, current_time):
        """
        This controller uses a fixed cycle time. At the start of each cycle it measures pressures
        and computes green durations proportional to the measured pressures. Between greens a fixed
        loss (yellow) time is applied.
        """
        self.measurement_data["counter"] += 1
        if self.measurement_data["counter"] == self.params["measurement_period"]:
            self.measurement_data["counter"] = 0
            # record pressures
            self.measurement_data["pressures_hist"].append(
                self.intersection.get_queue_lengths_num_vehicles()
            )
            self.measurement_data["fsm_timer"] += 1
            # timer went up, time to start new cycle
            if self.measurement_data["fsm_timer"] == 0:
                # calculate new schedule
                self.measurement_data["schedule"] = self._compute_schedule_from_pressures()                   
                self.measurement_data["history_schedule"].append(
                    [current_time, self.measurement_data["schedule"]]
                )
                if (self.intersection.tl_id=="intersection1"):
                    print(self.measurement_data["schedule"] )
                # set new schedule to traffic light
                self.intersection.apply_tl_programme(self.measurement_data["schedule"], self.params["T_L"])
                # wait until cycle complete
                self.measurement_data["fsm_timer"] = -self.params["cycle_duration"]+1