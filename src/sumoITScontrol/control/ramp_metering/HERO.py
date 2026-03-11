"""sumoITScontrol: Traffic Controller Collection for SUMO Traffic Simulations [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

import numpy as np


class HERO:
    def __init__(self, params, coordination_group, alinea_controllers):
        """
        coordination_group : RampMeterCoordinationGroup
        alinea_controllers  : dict {ramp_id: ALINEA instance}
        """
        self.params = params
        self.group = coordination_group
        self.alinea = alinea_controllers
        self.hero_counter = 0
        # HERO logging
        self.log = {"time": [], "ramps": {}}  # ramp_id -> dict of timeseries
        for ramp in coordination_group.ramp_meters:
            self.log["ramps"][ramp.tl_id] = {
                "role": [],  # NONE / MASTER / SLAVE
                "queue_length": [],
                "metering_rate": [],
                "controller": [],  # ALINEA / QUEUE
            }

    def _get_ramp_role(self, ramp):
        if not self.group.is_active():
            return "NONE"
        if ramp == self.group.get_master():
            return "MASTER"
        if ramp in self.group.get_slaves():
            return "SLAVE"
        return "NONE"

    def _log_state(self, current_time):
        self.log["time"].append(current_time)
        for ramp in self.group.ramp_meters:
            role = self._get_ramp_role(ramp)
            queue_len = self._get_queue_length(ramp)
            ramp_log = self.log["ramps"][ramp.tl_id]
            ramp_log["role"].append(role)
            ramp_log["queue_length"].append(queue_len)
            if role == "SLAVE":
                ramp_log["controller"].append("QUEUE")
                ramp_log["metering_rate"].append(ramp.current_metering_rate)
            else:
                ramp_log["controller"].append("ALINEA")
                ramp_log["metering_rate"].append(
                    self.alinea[ramp.tl_id].measurement_data["previous_metering_rate"]
                )

    def _get_queue_length(self, ramp):
        queue_state = ramp._get_queue_state("getLastIntervalMaxJamLengthInMeters")
        return np.nanmean(queue_state)

    def _get_occupancy(self, ramp):
        mainline_state = ramp._get_mainline_state("getLastIntervalOccupancy")
        return np.nanmean(mainline_state)

    def minimum_queue_control(self, slave_ctrl):
        """
        Deadbeat-style minimum queue controller (simplified)
        """
        ramp = slave_ctrl.ramp_meter
        T = slave_ctrl.params["cycle_duration"]
        N_hat_m = np.nanmean(
            ramp._get_queue_state("getLastIntervalMaxJamLengthInMeters")
        )
        N_hat_veh = N_hat_m / self.params["avg_vehicle_spacing"]
        N_max_m = self.params["min_queue_setpoint_m"]
        N_max_veh = N_max_m / self.params["avg_vehicle_spacing"]
        f = self.params["anticipation_factor"]
        flow_in_sm = np.nanmean(ramp._get_queue_state("getLastIntervalVehicleNumber"))
        # calculate desired discharge flow flow_qc
        flow_qc_per_cycle = N_max_veh - N_hat_veh + f * flow_in_sm
        flow_qc_per_sec = flow_qc_per_cycle / T
        green_fraction = flow_qc_per_sec / ramp.saturation_flow_veh_per_sec
        green_fraction = np.clip(
            green_fraction, slave_ctrl.params["min_rate"], slave_ctrl.params["max_rate"]
        )
        return green_fraction

    def execute_control(self, current_time):
        # 1. Always run local ALINEA
        for ctrl in self.alinea.values():
            ctrl.execute_control(current_time)
        # conduct hero
        skip_hero = False
        self.hero_counter += 1
        if self.hero_counter < self.params["hero_cycle_duration"]:
            skip_hero = True
        else:
            self.hero_counter = 0
        if not skip_hero:
            # 2. Check for HERO activation
            if not self.group.is_active():
                for idx, ramp_id in enumerate(self.group.ramp_meter_ids):
                    ramp = self.alinea[ramp_id].ramp_meter
                    queue_len = self._get_queue_length(ramp)
                    if queue_len > self.params["queue_activation_threshold_m"]:
                        self.group.activate_master(idx)
                        break
            # 3. HERO active -> recruit slaves if needed
            if self.group.is_active():
                master_id = self.group.get_master_id()
                master_ramp = self.alinea[master_id].ramp_meter
                master_queue = self._get_queue_length(master_ramp)
                if master_queue > self.params["queue_activation_threshold_m"]:
                    self.group.recruit_next_slave()
                # 4. Override slaves with minimum queue control
                for slave_id in self.group.get_slave_ids():
                    slave_ctrl = self.alinea[slave_id]
                    meter_rate_slave = self.minimum_queue_control(slave_ctrl)
                    slave_ctrl.ramp_meter._update_ramp_signal_control_logic_alinea(
                        metering_rate=meter_rate_slave,
                        cycle_duration=slave_ctrl.params["cycle_duration"],
                    )
                # 5. Dissolution condition
                if master_queue < self.params["queue_release_threshold_m"]:
                    self.group.release_all()
        # logging
        self._log_state(current_time)
