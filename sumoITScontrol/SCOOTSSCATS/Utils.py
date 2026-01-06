# #############################################################################
# ####### FairSCOSCA: Fairness At Arterial Signals - Just Around The Corner
# #######   AUTHOR:       Kevin Riehl <kriehl@ethz.ch>, Justin Weiss <juweiss@ethz.ch> 
# #######                 Anastasios Kouvelas <kouvelas@ethz.ch>, Michail A. Makridis <mmakridis@ethz.ch>
# #######   YEAR :        2025
# #######   ORGANIZATION: Traffic Engineering Group (SVT), 
# #######                 Institute for Transportation Planning and Systems,
# #######                 ETH Zürich
# #############################################################################

"""
    This script contains utils used for measurements, calculation of degree of 
    saturation, queue lengths, efficiency and equity measures, etc.
"""



# #############################################################################
# ###### IMPORTS ##############################################################
# #############################################################################
import traci
import numpy as np




# #############################################################################
# ###### GLOBAL VARIABLES #####################################################
# #############################################################################
# Global Variables
DS_SCATS = {}
runde = 0
T_NO = {}
T_NO_LAST = {}
greentime = {}
vehicles_count = {}
detected_vehicles = {}
lane_to_detector = {}
tracked_vehiclesIN = {}
last_vehicles_average = set()
last_vehicles_total = set()
vehicle_departure_times = {}
vehicle_waiting_times_average = {}
vehicle_waiting_times_average_sideroad = {}
vehicle_waiting_times_average_mainroad = {}
vehicle_departure_lanes = {}
waiting_times = {}
dist = {}
sideroad_lanes = set([
    '-183419042#1','-208691154#0','-25576697#0','-25973410#1','-E16',
    '283020993#1','758088375#0','-1169441386','-23999291#1','1162834479#1',
    '-E4','-281956705#3','384338154#2','-183419042#9','23320502','-23320456#1',
    '-394114218#1','-394114218#3','89290458#1',
    '25497525','-60430429#1','E0'
])
mainroad_lanes = set(['-37181834#3','921020465#1','E13','#183049957#0'])
# Lanes to Phases:
lane_to_phases = {
    "intersection1": {
        "921020465#1_3": [0],
        "921020465#1_2": [0],
        "921020464#0_1": [0],
        "921020464#1_1": [0],
        "38361907_3": [0],
        "38361907_2": [0],
        "-1164287131#1_3": [0],
        "-1164287131#1_2": [0],
        "-1169441386_2": [2],
        "-1169441386_1": [2],
        "-331752492#1_2": [2],
        "-331752492#1_1": [2],
        "-331752492#0_1": [2],
        "-331752492#0_2": [2],
        "-183419042#1_1": [4],
        "26249185#30_1": [4],
        "26249185#30_2": [4],
        "26249185#1_1": [4],
        "26249185#1_2": [4],
    },
    "intersection2": {
        "183049933#0_1": [0],
        "-38361908#1_1": [0, 2],
        "-38361908#1_2": [2],
        "-25973410#1_1": [4],
        "758088375#0_1": [4],
        "758088375#0_2": [4],
    },
    "intersection3": {
        "E3_1": [0, 2],
        "-758088377#1_1": [0, 4],
        "-758088377#1_2": [0],
        "-E1_1": [0, 4],
        "-E1_2": [0],
        "E3_2": [2],
        "-E4_1": [4],
        "-E4_2": [4],
    },
    "intersection4": {
        "22889927#0_1": [0],
        "758088377#2_1": [0],
        "-22889927#2_1": [0],
        "-25576697#0_0": [2],
    },
    "intersection5": {
        "E6_1": [0],
        "E6_2": [0],
        "E5_1": [0],
        "130569446_1": [0],
        "E15_1": [0],
        "E15_2": [0, 2],
        "E6_3": [2],
        "E5_2": [2],
        "130569446_2": [2],
        "E10_1": [4],
        "E9_1": [4],
        "1162834479#1_1": [4],
        "-208691154#0_1": [4],
        "-208691154#1_1": [4],
    },
}




# #############################################################################
# ###### METHODS ##############################################################
# #############################################################################

def get_lane_detectors():
    """
    Maps each lane to its corresponding detector.
    """
    global lane_to_detector
    all_detectors = traci.inductionloop.getIDList()
    for detector_id in all_detectors:
        lane_id = traci.inductionloop.getLaneID(detector_id)
        if lane_id:
            lane_to_detector[lane_id] = detector_id
    return lane_to_detector


def calculate_degree_of_saturation_SCATS(greentimes, cyclelength, step, JUNCTION_IDS, lanes):
    """
    Calculates the degree of saturation (DS) similar to SCATS.
    """
    global runde, vehicles_count, T_NO, T_NO_LAST, greentime, DS_SCATS, detected_vehicles
    if step == 0:
        runde = 0
        get_lane_detectors()
    if runde == 0:
        T_NO = {}
        greentime = {}
        vehicles_count = {}
        T_NO_LAST = {}
        DS_SCATS = {}
        detected_vehicles = {}
        for junction in JUNCTION_IDS:
            greentime[junction] = {}
            T_NO[junction] = {}
            DS_SCATS[junction] = {}
            T_NO_LAST[junction] = {}
            vehicles_count[junction] = {}
            detected_vehicles[junction] = {}
            for lane in lanes[junction]:
                if lane in lane_to_detector:
                    vehicles_count[junction][lane] = 0
                    T_NO_LAST[junction][lane] = 0
                    detected_vehicles[junction][lane] = set()
                    phases = lane_to_phases[junction][lane]
                    greentime[junction][lane] = sum([greentimes[junction][p//2] for p in phases])
                    T_NO[junction][lane] = greentime[junction][lane]
    for junction in JUNCTION_IDS:
        phase = traci.trafficlight.getPhase(junction)
        for lane in lanes[junction]:
            if lane in lane_to_detector:
                detector = lane_to_detector[lane]
                if phase in lane_to_phases[junction][lane]:
                    current_vehicles = set(traci.inductionloop.getLastStepVehicleIDs(detector))
                    new_vehicles = current_vehicles - detected_vehicles[junction][lane]
                    occupancy = traci.inductionloop.getLastStepOccupancy(detector) / 100
                    if occupancy > 0:
                        T_NO[junction][lane] -= occupancy
                        vehicles_count[junction][lane] += len(new_vehicles)
                        detected_vehicles[junction][lane].update(new_vehicles)
    if runde == cyclelength - 1:
        for junction in JUNCTION_IDS:
            for lane in lanes[junction]:
                if lane in lane_to_detector:
                    DS_SCATS[junction][lane] = max(
                        (greentime[junction][lane] - (T_NO[junction][lane] - 1.87 * vehicles_count[junction][lane]))
                        / greentime[junction][lane],
                        0,
                    )
        runde = 0
        return DS_SCATS
    runde += 1
    return None


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


def get_throughput(lanes,step):
    """
    Tracks throughput for every controlled junction summed together.
    """
    global tracked_vehiclesIN
    total_throughput = 0
    if step==1800:
        get_lane_detectors()
        for j in lanes.keys():
            for lane in lanes[j]:
                if lane in lane_to_detector:
                    tracked_vehiclesIN[lane] = set()
        return 0  # No throughput in first step
    for j in lanes.keys():
        for lane in lanes[j]:
            if lane in lane_to_detector:
                current_vehicles = set(traci.lane.getLastStepVehicleIDs(lane))
                passed_vehicles = tracked_vehiclesIN[lane] - current_vehicles
                total_throughput += len(passed_vehicles)
                tracked_vehiclesIN[lane] = current_vehicles
    return total_throughput


def get_flow():
    """
    Tracks network flow.
    """
    global last_vehicles_total
    current_vehicles = set(traci.vehicle.getIDList())
    total_flow = len(last_vehicles_total - current_vehicles)
    return total_flow


def get_total_distance():
    """
    Tracks the total distance (used for average speed).
    """
    global dist
    total_distance = 0
    current_veh = set(traci.vehicle.getIDList())
    for veh_id in current_veh:
        dist[veh_id] = traci.vehicle.getDistance(veh_id)
    for veh_id in last_vehicles_total - current_veh:
        total_distance += dist.pop(veh_id, 0)
    return total_distance


def get_total_travel_time(step):
    """
    Tracks Total Travel Time.
    """
    global last_vehicles_total, vehicle_departure_times
    total_travel_time = 0  
    current_vehicles = set(traci.vehicle.getIDList())
    for veh_id in current_vehicles:
        vehicle_departure_times[veh_id] = traci.vehicle.getDeparture(veh_id)
    # Loop through all vehicles that left simulation
    for veh_id in last_vehicles_total - current_vehicles:
        total_travel_time += step - vehicle_departure_times.pop(veh_id,0)
    last_vehicles_total = current_vehicles
    #Return the total waiting time in this time step (waiting time of all vehicles that finished in this time step)
    return total_travel_time


def get_density():
    """
    Tracks Density.
    """
    return len(traci.vehicle.getIDList())


def get_max_delay():
    """
    Tracks Max Delay.
    """
    max_delay = max(vehicle_waiting_times_average.values())
    return max_delay 


def get_average_delay_total():
    """
    Tracks all avgerage delays.
    """
    global last_vehicles_average, vehicle_waiting_times_average
    global vehicle_departure_lanes, vehicle_waiting_times_average_mainroad
    global vehicle_waiting_times_average_sideroad
    total_waiting_time = 0
    vehicle_count = 0
    total_waiting_time_sideroad = 0
    vehicle_count_sideroad = 0
    total_waiting_time_mainroad = 0
    vehicle_count_mainroad = 0
    current_vehicles = set(traci.vehicle.getIDList())
    for veh_id in current_vehicles:
        vehicle_waiting_times_average[veh_id] = traci.vehicle.getAccumulatedWaitingTime(veh_id)
        if veh_id not in vehicle_departure_lanes:
            route = traci.vehicle.getRoute(veh_id)
            if route:
                vehicle_departure_lanes[veh_id] = route[0]
    if last_vehicles_average is not None:
        departed_vehicles = last_vehicles_average - current_vehicles
        for veh_id in departed_vehicles:
            wait_time = vehicle_waiting_times_average.get(veh_id, 0)
            lane = vehicle_departure_lanes.get(veh_id)
            total_waiting_time += wait_time
            vehicle_count += 1
            if lane in sideroad_lanes:
                vehicle_waiting_times_average_sideroad[veh_id] = wait_time
                total_waiting_time_sideroad += wait_time
                vehicle_count_sideroad += 1
            else:
                vehicle_waiting_times_average_mainroad[veh_id] = wait_time
                total_waiting_time_mainroad += wait_time
                vehicle_count_mainroad += 1
    last_vehicles_average = current_vehicles
    return (total_waiting_time, vehicle_count, total_waiting_time_sideroad, 
            vehicle_count_sideroad, total_waiting_time_mainroad, vehicle_count_mainroad) 


def get_gini():
    """
    Tracks Gini coefficient of vehicle delays.
    """
    total = np.array(list(vehicle_waiting_times_average.values()))
    sideroad = np.array(list(vehicle_waiting_times_average_sideroad.values()))
    mainroad = np.array(list(vehicle_waiting_times_average_mainroad.values()))
    
    L = [total, sideroad, mainroad]
    gini_index = []
    for i in L:
        n = len(i)
        mean_value = np.mean(i)
        total_absolute_differences = np.sum(np.abs(i[:, None] - i))  # Pairwise differences
        gini_index.append(total_absolute_differences / (2 * n**2 * mean_value))
    return gini_index  


def get_waiting_times(cyclelength, lanes, up_stream_lanes, df_hidden_vehicles):
    """
    Calculates the waiting time per lane (sum of all waiting vehicles).
    """
    global waiting_times
    if runde == 0:
        for junction in up_stream_lanes.keys():
            waiting_times[junction] = {}
            for lane in lanes[junction]:
                waiting_times[junction][lane] = 0  
        return None
    for junction in up_stream_lanes.keys():
        phase = traci.trafficlight.getPhase(junction)
        # Add hidden vehicles based on internal edge location
        if df_hidden_vehicles is not None and "edge" in df_hidden_vehicles.columns:
            for lane in lanes[junction]:
                edge = lane.split("_")[0]
                hidden_on_edge = df_hidden_vehicles[df_hidden_vehicles["edge"] == edge]
                if phase not in lane_to_phases[junction][lane]:
                    for _, row in hidden_on_edge.iterrows():
                        veh = row["veh_id"]  # or replace with actual column name
                        if traci.vehicle.getSpeed(veh) < 0.1 and traci.vehicle.getWaitingTime(veh) > 0:
                            waiting_times[junction][lane] += 1                 
        # Only count waiting time if signal is red (or yellow)
        for lane in lanes[junction]:
            if phase not in lane_to_phases[junction][lane]:
                veh_ids = traci.lane.getLastStepVehicleIDs(lane)
                for veh_id in veh_ids:
                    if traci.vehicle.getSpeed(veh_id) < 0.1 and traci.vehicle.getWaitingTime(veh_id) > 0:
                        waiting_times[junction][lane] += 1
    if runde == cyclelength - 1:
        # Add upstream vehicles to downstream lane
        for junction in up_stream_lanes.keys():
            for down, ups in up_stream_lanes[junction].items():
                for up in ups:
                    waiting_times[junction][down] += waiting_times[junction].get(up, 0)
        return waiting_times
    else:
        return None

