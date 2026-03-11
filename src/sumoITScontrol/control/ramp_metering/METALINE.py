"""sumoITScontrol: Traffic Controller Collection for SUMO Traffic Simulations [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

import numpy as np


class METALINE:
    def __init__(self, params, coordination_group, target_occupancies, K_P, K_I=None):
        """
        params: dict
            - cycle_duration: duration of control cycle in seconds
        coordination_group: RampMeterCoordinationGroup
        target_occupancies: list or array of target occupancies per ramp
        K_P: np.array, n x n proportional gain matrix
        K_I: np.array, n x n integral gain matrix (optional, default zero)
        """
        self.params = params
        self.group = coordination_group
        self.n_ramps = len(coordination_group.ramp_meters)
        # Gains
        self.K_P = np.array(K_P)
        self.K_I = np.zeros_like(self.K_P) if K_I is None else np.array(K_I)
        # Target occupancies
        self.target_occupancies = np.array(target_occupancies)
        # Initialize previous states
        self.prev_occupancies = np.zeros(self.n_ramps)
        self.prev_rates = np.ones(self.n_ramps) * 100.0  # start fully open
        # Measurement data
        self.measurement_data = {
            rm_id: {"occupancy": [], "queue_length_m": [], "metering_rate": []}
            for rm_id in coordination_group.ramp_meter_ids
        }
        # Counter for measurement period
        self.counter = -1

    def execute_control(self, current_time):
        self.counter += 1
        if self.counter == self.params["measurement_period"]:
            self.counter = 0
            # Collect measurements
            occupancies = np.zeros(self.n_ramps)
            queues = np.zeros(self.n_ramps)
            for i, ramp_meter in enumerate(self.group.ramp_meters):
                occupancies[i] = np.nanmean(
                    ramp_meter._get_mainline_state("getLastIntervalOccupancy")
                )
                queues[i] = np.nanmean(
                    ramp_meter._get_queue_state("getLastIntervalMaxJamLengthInMeters")
                )
            # METALINE control law (vectorized)
            rates = (
                self.prev_rates
                + self.K_P @ (self.target_occupancies - occupancies)
                + self.K_I @ (occupancies - self.prev_occupancies)
            )
            # Saturate to min/max
            rates = np.clip(
                rates, self.params.get("min_rate", 5), self.params.get("max_rate", 100)
            )
            # Apply metering
            for i, ramp_meter in enumerate(self.group.ramp_meters):
                ramp_meter._update_ramp_signal_control_logic_alinea(
                    metering_rate=rates[i] / 100.0,  # convert to fraction
                    cycle_duration=self.params["cycle_duration"],
                )
                # record data
                rm_id = self.group.ramp_meter_ids[i]
                self.measurement_data[rm_id]["occupancy"].append(
                    [current_time, occupancies[i]]
                )
                self.measurement_data[rm_id]["queue_length_m"].append(
                    [current_time, queues[i]]
                )
                self.measurement_data[rm_id]["metering_rate"].append(
                    [current_time, rates[i]]
                )
            # Update previous states
            self.prev_rates = rates
            self.prev_occupancies = occupancies
