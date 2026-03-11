"""sumoITScontrol: Traffic Controller Collection for SUMO Traffic Simulations [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

class RampMeterCoordinationGroup:
    def __init__(self, ramp_meters_ordered, ramp_meter_ids):
        """
        ramps: ordered list of RampMeter objects: downstream -> upstream
        """
        self.ramp_meters = ramp_meters_ordered
        self.ramp_meter_ids = ramp_meter_ids
        self.master_idx = None
        self.slave_indices = []

    def is_active(self):
        return self.master_idx is not None

    def activate_master(self, idx):
        self.master_idx = idx
        self.slave_indices = []

    def recruit_next_slave(self):
        if self.master_idx is None:
            return None
        next_idx = self.master_idx + len(self.slave_indices) + 1
        if next_idx < len(self.ramp_meters):
            self.slave_indices.append(next_idx)
            return self.ramp_meters[next_idx]
        return None

    def release_all(self):
        self.master_idx = None
        self.slave_indices = []

    def get_master(self):
        if self.master_idx is None:
            return None
        return self.ramp_meters[self.master_idx]

    def get_slaves(self):
        return [self.ramp_meters[i] for i in self.slave_indices]

    def get_master_id(self):
        if self.master_idx is None:
            return None
        return self.ramp_meter_ids[self.master_idx]

    def get_slave_ids(self):
        return [self.ramp_meter_ids[i] for i in self.slave_indices]
