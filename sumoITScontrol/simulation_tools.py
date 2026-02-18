class SimulationTools:
    # sensor related
    sensor_list_initialized = False
    sensor_list_e1_inductionloops = []
    sensor_list_e2_laneareas = []
    # hidden vehicle related
    hidden_vehicle_update_time = -1
    hidden_vehicles = []
    hidden_vehicles_current_edge = []
    # length related
    lane_lengths = {}

    @staticmethod
    def init_sensor_lists(traci):
        SimulationTools.sensor_list_e1_inductionloops = traci.inductionloop.getIDList()
        SimulationTools.sensor_list_e2_laneareas = traci.lanearea.getIDList()
        SimulationTools.sensor_list_initialized = True

    @staticmethod
    def get_sensor_type(sensor_id: str) -> str | None:
        if sensor_id in SimulationTools.sensor_list_e1_inductionloops:
            return "inductionloop"
        if sensor_id in SimulationTools.sensor_list_e2_laneareas:
            return "lanearea"
        return None

    @staticmethod
    def determine_hidden_vehicles(traci):
        if (
            SimulationTools.hidden_vehicle_update_time == -1
            or SimulationTools.hidden_vehicle_update_time
            != traci.simulation.getCurrentTime()
        ):
            SimulationTools.hidden_vehicle_update_time = (
                traci.simulation.getCurrentTime()
            )
            current_vehicles = traci.vehicle.getIDList()
            current_lanes = [traci.vehicle.getLaneID(v_id) for v_id in current_vehicles]
            SimulationTools.hidden_vehicles = [
                v_id
                for v_id, lane_id in zip(current_vehicles, current_lanes)
                if lane_id.startswith(":")
            ]
            hidden_vehicles_routes = [
                traci.vehicle.getRoute(v_id) for v_id in SimulationTools.hidden_vehicles
            ]
            hidden_vehicles_routes_index = [
                traci.vehicle.getRouteIndex(v_id)
                for v_id in SimulationTools.hidden_vehicles
            ]
            # SimulationTools.hidden_vehicles_current_edge = [hidden_vehicles_routes[hidden_vehicles_routes_index[v_id]] for v_id in SimulationTools.hidden_vehicles]
            SimulationTools.hidden_vehicles_current_edge = [
                hidden_vehicles_routes[i][hidden_vehicles_routes_index[i]]
                for i in range(len(SimulationTools.hidden_vehicles))
            ]

    @staticmethod
    def get_lane_length(traci, lane_id):
        if not lane_id in SimulationTools.lane_lengths:
            SimulationTools.lane_lengths[lane_id] = traci.lane.getLength(lane_id)
        return SimulationTools.lane_lengths.get(lane_id)

    @staticmethod
    def get_lane_length_preloaded(lane_id):
        return SimulationTools.lane_lengths.get(lane_id)
