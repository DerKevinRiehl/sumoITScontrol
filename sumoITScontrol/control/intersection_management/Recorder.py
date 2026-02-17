import traci

class Recorder:
    
    def get_queue_lengths(lanes,up_stream_lanes, df_hidden_vehicles):
        """
        Tracks queue lengths per lane, including upstream contributions.
        """
        queue_lengths = {}
        for j in up_stream_lanes.keys():
            queue_lengths[j] = {}
            # Initialize all involved lanes to 0 first
            for lane in lanes[j]:
                queue_lengths[j][lane] = 0
            # Add hidden vehicles based on internal edge location
            if df_hidden_vehicles is not None and "edge" in df_hidden_vehicles.columns:
                for lane in lanes[j]:
                    edge = lane.split("_")[0]
                    hidden_on_edge = df_hidden_vehicles[df_hidden_vehicles["edge"] == edge]
                    queue_lengths[j][lane] += len(hidden_on_edge)
    
            # Add direct vehicles on every lane
            for lane in queue_lengths[j].keys():
                queue_lengths[j][lane] += traci.lane.getLastStepVehicleNumber(lane)
    
            # Add upstream vehicles to downstream lane
            for down, ups in up_stream_lanes[j].items():
                for up in ups:
                    queue_lengths[j][down] += queue_lengths[j].get(up, 0)
        return queue_lengths