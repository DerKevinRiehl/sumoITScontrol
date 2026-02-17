import traci
from .simulation_tools import SimulationTools

class IntersectionGroup:
    def __init__(self, intersections, speed_limit=3.66, districts=None, critical_district_order=None, connection_between_intersections=None):
        self.intersections = intersections  
        self.speed_limit = speed_limit
        self.districts = districts
        self.critical_district_order = critical_district_order
        self.connection_between_intersections = connection_between_intersections
        
    def _init_lane_lengths(self):
        edge_ids = traci.edge.getIDList()
        for edge_id in edge_ids:
            for lane_idx in range(0, traci.edge.getLaneNumber(edge_id)):
                lane_id = edge_id+"_"+str(lane_idx)
                SimulationTools.get_lane_length(traci, lane_id)       
        
    def get_intersection(self, tl_id):
        for intersection in self.intersections:
            if intersection.tl_id == tl_id:
                return intersection
        
    def apply_tl_programme(self, greentimes, offsets):
        for intersection in self.intersections:
            junction = intersection.tl_id
            greens = greentimes.get(junction, [])
            if not greens:
                continue
            phases = []
            for idx, g in enumerate(greens):
                gstate = intersection.green_states[idx] if idx < len(intersection.green_states) else "G" * max(1, len(intersection.phases))
                ystate = intersection.yellow_states[idx] if idx < len(intersection.yellow_states) else "y" * len(gstate)
                phases.append(traci.trafficlight.Phase(int(g), gstate))
                phases.append(traci.trafficlight.Phase(3, ystate))
            # logic
            logic = traci.trafficlight.Logic(programID=f"program_fixed_{junction}", 
                                             type=traci.tc.TRAFFICLIGHT_TYPE_STATIC, 
                                             currentPhaseIndex=0, 
                                             phases=phases)
            traci.trafficlight.setProgramLogic(junction, logic)
            # offsets
            shift = int(offsets.get(junction, 0))
            if shift == 0:
                traci.trafficlight.setPhase(junction, 0)
            elif shift < 3:
                traci.trafficlight.setPhase(junction, len(greens) * 2 - 1)
                traci.trafficlight.setPhaseDuration(junction, shift)
            elif shift < greens[-1] + 3:
                traci.trafficlight.setPhase(junction, len(greens) * 2 - 2)
                traci.trafficlight.setPhaseDuration(junction, shift - 3)
            elif shift < greens[-1] + 6:
                traci.trafficlight.setPhase(junction, len(greens) * 2 - 3)
                traci.trafficlight.setPhaseDuration(junction, shift - greens[-1] - 3)
            elif shift < greens[-1] + 6 + (greens[-2] if len(greens) > 1 else 0):
                traci.trafficlight.setPhase(junction, len(greens) * 2 - 4)
                traci.trafficlight.setPhaseDuration(junction, shift - greens[-1] - 6)
            else:
                if len(greens) * 2 > 4:
                    if shift < greens[-1] + 9 + (greens[-2] if len(greens) > 1 else 0):
                        traci.trafficlight.setPhase(junction, len(greens) * 2 - 5)
                        traci.trafficlight.setPhaseDuration(junction, shift - greens[-1] - (greens[-2] if len(greens) > 1 else 0) - 6)
                    elif shift < greens[-1] + 9 + (greens[-2] if len(greens) > 1 else 0) + (greens[-3] if len(greens) > 2 else 0):
                        traci.trafficlight.setPhase(junction, len(greens) * 2 - 6)
                        traci.trafficlight.setPhaseDuration(junction, shift - greens[-1] - (greens[-2] if len(greens) > 1 else 0) - 9)
                    else:
                        print("SCOSCA OFFSET ERROR", junction, offsets)
                else:
                    print("SCOSCA OFFSET ERROR SMALL", junction, offsets)

    def measure_queues_and_ds(self):
        """
        Return tuple (queue_lengths, degree_of_sat)
        - queue_lengths: dict intersection -> dict lane -> vehicle_count
        - degree_of_sat: dict intersection -> dict lane -> ds (0..1)
        """
        queue_lengths = {}
        degree_of_sat = {}
        for intersection in self.intersections:
            tl = intersection.tl_id
            queue_lengths[tl] = {}
            degree_of_sat[tl] = {}
            # get list of lanes from links provided on Intersection
            lane_candidates = []
            # links is a dict phase->list(lanes)
            for lanes in intersection.links.values():
                lane_candidates.extend(lanes)
            # deduplicate
            lane_candidates = list(dict.fromkeys(lane_candidates))
            for lane in lane_candidates:
                try:
                    # use last-step vehicle number as queue estimate
                    veh = traci.lane.getLastStepVehicleNumber(lane)
                    length = max(1.0, SimulationTools.get_lane_length_preloaded(lane))
                except traci.TraCIException:
                    # if lane not recognized (e.g. sensor names), fallback to zero
                    veh = 0
                    length = 10.0
                queue_lengths[tl][lane] = int(veh)
                # simple DS estimator: vehicles per 7m of lane (approx. vehicle length + gap), clipped
                ds = min(1.0, float(veh) / max(1.0, (length / 7.0)))
                degree_of_sat[tl][lane] = float(ds)
        return queue_lengths, degree_of_sat