# sumoITScontrol/control/intersection_management/MaxPressure_Fix.py


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

    def _compute_schedule_from_pressures(self):
        n = len(self.intersection.phases)
        # calculate average pressure during last cycle
        self.measurement_data["pressures"] = [
            sum(window) / len(window)
            for window in self.measurement_data["pressures_hist"]
        ]
        self.measurement_data["pressures_hist"] = []
        # effective green time available after losses
        total_loss = n * self.params["T_L"]
        effective_green = max(
            0.0, float(self.params["cycle_duration"]) - float(total_loss)
        )
        total_pressure = (
            sum(self.measurement_data["pressures"])
            if self.measurement_data["pressures"]
            else 0
        )
        greens = []
        if total_pressure <= 0:
            # distribute equally
            base = effective_green / n if n > 0 else 0
            greens = [base for _ in range(n)]
        else:
            for p in self.measurement_data["pressures"]:
                share = float(p) / float(total_pressure)
                greens.append(share * effective_green)
        # enforce min and max and integer seconds
        greens = [
            max(self.params["G_T_MIN"], min(g, self.params["G_T_MAX"])) for g in greens
        ]
        # if sum exceeds effective_green because of G_T_MIN, fall back to equal split of effective_green
        total_alloc = sum(greens)
        if total_alloc > effective_green and effective_green > 0:
            equal = effective_green / n
            greens = [
                max(self.params["G_T_MIN"], min(equal, self.params["G_T_MAX"]))
                for _ in greens
            ]
        # final check: if effective_green is zero (cycle smaller than total loss), set tiny greens
        if effective_green == 0:
            greens = [0 for _ in range(n)]
        # round to nearest integer second to avoid fractional timing issues
        greens = [int(max(0, round(g))) for g in greens]
        # ensure at least G_T_MIN where possible: if effective_green small, we may need to adjust to fit cycle
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
            self.measurement_data["fsm_timer"] += 1
            self.measurement_data["pressures_hist"].append(
                self.intersection.get_queue_lengths_num_vehicles()
            )
            # when starting a new cycle (schedule index 0 and not currently in green), compute schedule
            if self.measurement_data["schedule_index"] == 0 and self.measurement_data[
                "current_fsm_state"
            ] in ("idle", "transition"):
                self.measurement_data["schedule"] = (
                    self._compute_schedule_from_pressures()
                )
            # if FSM is idle, start first green
            if self.measurement_data["current_fsm_state"] == "idle":
                # start green for current schedule index
                idx = self.measurement_data["schedule_index"]
                phase = self.intersection.phases[idx]
                self.measurement_data["current_signal_phase"] = phase
                self.measurement_data["current_gt_start"] = current_time
                self.measurement_data["fsm_timer"] = 0
                self.measurement_data["current_fsm_state"] = "green"
                # apply
                self.intersection.set_signal_on_traffic_lights(phase=int(phase))
                return
            # If in green, check if it should end
            if self.measurement_data["current_fsm_state"] == "green":
                idx = self.measurement_data["schedule_index"]
                schedule = self.measurement_data.get("schedule", [])
                green_duration = (
                    schedule[idx] if idx < len(schedule) else self.params["G_T_MIN"]
                )
                # fsm_timer counted in measurement windows -> convert to seconds approximation
                # measurement_period is number of sim steps per measurement; assume measurement_period corresponds to 1s in demo config
                # To be robust, compare simulation absolute time
                current_gt = current_time - self.measurement_data["current_gt_start"]
                if current_gt >= green_duration:
                    # go to yellow (transition)
                    # yellow is typically the next phase index
                    self.measurement_data["next_signal_phase"] = int(
                        self.measurement_data["current_signal_phase"] + 1
                    )
                    self.measurement_data["current_signal_phase"] = (
                        self.measurement_data["current_signal_phase"] + 1
                    )
                    self.measurement_data["fsm_timer"] = 0
                    self.measurement_data["current_fsm_state"] = "transition"
                    # apply yellow
                    self.intersection.set_signal_on_traffic_lights(
                        phase=int(self.measurement_data["current_signal_phase"])
                    )
                    return
                else:
                    # keep green
                    self.intersection.set_signal_on_traffic_lights(
                        phase=int(self.measurement_data["current_signal_phase"])
                    )
                    return
            # If in transition (yellow), check if yellow elapsed
            if self.measurement_data["current_fsm_state"] == "transition":
                current_t = self.measurement_data["fsm_timer"] * (
                    1
                )  # timer counted in measurement ticks; treat as seconds
                if current_t >= self.params["T_L"]:
                    # advance to next schedule index
                    self.measurement_data["schedule_index"] += 1
                    if self.measurement_data["schedule_index"] >= len(
                        self.intersection.phases
                    ):
                        # cycle finished -> wrap and start idle for next cycle computation
                        self.measurement_data["schedule_index"] = 0
                        self.measurement_data["current_fsm_state"] = "idle"
                        self.measurement_data["fsm_timer"] = -1
                        # start will be handled in next execute_control call
                    else:
                        # start next green
                        idx = self.measurement_data["schedule_index"]
                        phase = self.intersection.phases[idx]
                        self.measurement_data["current_signal_phase"] = phase
                        self.measurement_data["current_gt_start"] = current_time
                        self.measurement_data["fsm_timer"] = 0
                        self.measurement_data["current_fsm_state"] = "green"
                        self.intersection.set_signal_on_traffic_lights(phase=int(phase))
                else:
                    # remain in yellow
                    self.intersection.set_signal_on_traffic_lights(
                        phase=int(self.measurement_data["current_signal_phase"])
                    )
                    return
            # safety fallback: ensure traffic light is set
            self.intersection.set_signal_on_traffic_lights(
                phase=int(self.measurement_data["current_signal_phase"])
            )
