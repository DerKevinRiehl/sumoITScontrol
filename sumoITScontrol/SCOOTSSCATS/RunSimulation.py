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
    This code runs a SUMO traffic microsimulation with a specific traffic light
    controller, collects data during runtime, and generates log files.
"""




# #############################################################################
# ###### IMPORTS ##############################################################
# #############################################################################
import os
import sys
import traci
import time
import pandas as pd
import numpy as np
import random
import warnings
from datetime import datetime, timedelta
from Utils import (calculate_degree_of_saturation_SCATS,get_throughput,
                    get_average_delay_total, get_queue_lengths,get_total_travel_time,
                    get_flow,get_total_distance,get_density,get_max_delay,get_gini, get_waiting_times)
from ControllerMaxPressure import WEIGHTS_MAX_PRESSURE, signal_controllers
from ControllerSCOSCA import setup_scosca_control
from ControllerFairSCOSCA_1 import setup_scoscafairv1_control
from ControllerFairSCOSCA_2 import setup_scoscafairv2_control, Optimizer_Fairness
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
warnings.filterwarnings("ignore")





# #############################################################################
# ###### RUN ARGUMENTS PARSING ################################################
# #############################################################################
SUMO_BINARY = "C:/Users/juweiss/AppData/Local/sumo-1.22.0/bin/sumo.exe"  # Adjust if needed
CONTROL_MODE = "SCOSCA" # FIXED_CYCLE, MAX_PRESSURE, SCOSCA, SCOSCAFAIRV1, SCOSCAFAIRV2
sys.argv = ['RunSimulation.py',
            '--sumo-path', SUMO_BINARY,
            '--controller', CONTROL_MODE]
"""
SUMO_BINARY = "C:/Users/juweiss/AppData/Local/sumo-1.22.0/bin/sumo.exe"
SUMO_BINARY = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo-gui.exe"
CONTROL_MODE = "MAX_PRESSURE"
"""




# #############################################################################
# ###### SIMULATION PARAMETER #################################################
# #############################################################################
    # TIME PARAMETER
SIMULATION_STEPS_PER_SECOND = 1
SIMULATION_WAIT_TIME = 0
START_TIME = datetime.strptime("2024-03-04 15:15:00", "%Y-%m-%d %H:%M:%S")
END_TIME = datetime.strptime("2024-03-04 17:45:00", "%Y-%m-%d %H:%M:%S")
SIMULATION_TIMES = [dt.strftime("%Y-%m-%d %H:%M:%S") for dt in [START_TIME + timedelta(seconds=i) for i in range(int((END_TIME - START_TIME).total_seconds()) + 1)]]
SIMULATION_DURATION = int((END_TIME - START_TIME).total_seconds())
    # PUBLIC TRANSPORT PARAMETER
BUS_STOP_DURATION = 20 # SECS
    # DEBUGGING
DEBUG_CONTROLLER_LOG = "NONE"# "intersection2"
DEBUG_TIME = True
DEBUG_GUI = True




# #############################################################################
# ###### METHODS ##############################################################
# #############################################################################

def get_random_vehicle_class(no_truck=False):
    probs = [0.81, 0.082, 0.046, 0.062]
    vals = ["car", "moc", "lwt", "hwt"]
    random_vehicle_class = np.random.choice(vals, size=1, p=probs)[0]
    while no_truck and random_vehicle_class=="hwt":
        random_vehicle_class = np.random.choice(vals, size=1, p=probs)[0]
    return random_vehicle_class

def determine_whether_truck_banned_route(desired_route):
    route_entrance = desired_route.split("_")[1]
    route_exit = desired_route.split("_")[2]
    selected_entrances = ["E21", "E22", "E24", "E25", "E20", "E3", "E4", "E5", "E1", "E2", "E6", "E7", "E12", "E13"]
    selected_exits = ["A1", "A2", "A3", "A16", "A18", "A15"]
    if route_entrance in selected_entrances and route_exit in selected_exits:
        return False
    return True

sumo_vehicle_types = {
    "car": "sumo_car",
    "moc": "sumo_motorcycle",
    "lwt": "sumo_transporter",
    "hwt": "sumo_truck",
    "bus": "sumo_bus",
}

def spawn_random_vehicle(veh_ctr, desired_route):
    # determine vehicle characteristics
    new_vehicle_id = "VEH_"+str(veh_ctr)
    no_truck = determine_whether_truck_banned_route(desired_route)
    vehicle_class = get_random_vehicle_class(no_truck)
    vehicle_type = sumo_vehicle_types[vehicle_class]
    # add vehicle with traci
    traci.vehicle.add(new_vehicle_id, desired_route, typeID=vehicle_type)
    veh_routes[new_vehicle_id] = desired_route
    veh_classes[new_vehicle_id] = vehicle_class
    
def spawn_random_bus(veh_ctr, desired_route, stops):
    # determine vehicle characteristics
    new_vehicle_id = "BUS_"+str(veh_ctr)+"-"+desired_route
    vehicle_class = "bus"
    vehicle_type = sumo_vehicle_types[vehicle_class]
    # add vehicle with traci
    traci.vehicle.add(new_vehicle_id, desired_route, typeID=vehicle_type)
    for stop in stops.split("-"):
        traci.vehicle.setBusStop(new_vehicle_id, stop, duration=BUS_STOP_DURATION)    
    veh_routes[new_vehicle_id] = desired_route
    veh_classes[new_vehicle_id] = vehicle_class

def determine_current_state():
    current_vehicles = traci.vehicle.getIDList()
    if len(current_vehicles)==0:
        print(">> NOTHING, so no state")
        return None, None
    current_lanes = [traci.vehicle.getLaneID(v_id) for v_id in current_vehicles]
    new_current_lanes = []
    for v_ctr in range(0, len(current_vehicles)):
        if not current_lanes[v_ctr].startswith(":"):
            new_current_lanes.append(current_lanes[v_ctr])
        else:
            v_id = current_vehicles[v_ctr]
            v_route = traci.vehicle.getRoute(v_id)
            v_current_edge_index = traci.vehicle.getRouteIndex(v_id)
            v_current_edge = v_route[v_current_edge_index]
            new_current_lanes.append("@"+v_current_edge)
    df_current_status = pd.DataFrame(np.asarray([current_vehicles, new_current_lanes]).transpose(), columns=["veh_id", "lane"])
    df_current_status["class"] = df_current_status["veh_id"].map(veh_classes)
    if CONTROL_MODE=="MAX_PRESSURE":
        df_current_status["weight"] = df_current_status["class"].map(WEIGHTS_MAX_PRESSURE)
    df_hidden_vehicles = df_current_status[df_current_status["lane"].str.startswith("@")]
    df_hidden_vehicles["edge"] = df_hidden_vehicles["lane"].str.replace("@","")
    excluded_edges = {"921020464#1","-331752492#0","38361907","26249185#30","183049933#0","758088375#0","-38361908#1",
                      "-25973410#1","E3","-E1","-E4","22889927#0","-25576697#0","-22889927#2","-208691154#0",
                      "E15","E10","E6"}
    df_hidden_vehicles = df_hidden_vehicles[~df_hidden_vehicles["edge"].isin(excluded_edges)]
    return df_current_status, df_hidden_vehicles



    
# #############################################################################
# ###### MAIN CODE ############################################################
# #############################################################################
def Simulation(params):
    #Define Global Variables
    global veh_routes, veh_classes
    global df_current_status, df_hidden_vehicles
    global tracked_vehiclesIN
    global last_vehicles_average, vehicle_waiting_times_average
    global last_vehicles_total, vehicle_departure_times
    #Define Params
    seed, adaptation_cycle, adaptation_green, green_thresh, adaptation_offset, offset_thresh,alpha, Changetime, Thresholdtime = params
    #Add Randomness
    random.seed(seed)
    np.random.seed(seed)
    #Create Local Variables
    veh_routes = {}
    veh_classes = {}
    df_current_status = None
    df_hidden_vehicles = None
    tracked_vehiclesIN = {}
    last_vehicles_average = None
    vehicle_waiting_times_average = {}
    last_vehicles_total = set()
    vehicle_departure_times = {}
    #Introduce Junctions and Initial Values for Variables
    JUNCTION_IDS = [c.intersection_name for c in signal_controllers]
    lanes = {}
    for c in signal_controllers:
        lanes[c.intersection_name] = [lane for group in c.links.values() for lane in group]
    step = 0
    greentimes = {
        "intersection1": [27, 27, 27],     # phases: 0, 2, 4
        "intersection2": [38, 6, 37],      # phases: 0, 2, 4
        "intersection3": [38, 6, 37],      # phases: 0, 2, 4
        "intersection4": [42, 42],         # phases: 0, 2
        "intersection5": [38, 6, 37],      # phases: 0, 2, 4
    }
    up_stream_links = {
        "intersection1": {
        "921020464#1_1":{"921020464#0_1","921020465#1_2"},
        "-331752492#0_2":{"-331752492#1_2","-1169441386_2"},
        "-331752492#0_1":{"-331752492#1_1","-1169441386_1"},
        "38361907_3":{"-1164287131#1_3","-183049933#0_2"},
        "38361907_2":{"-1164287131#1_2","-183049933#0_1"},
        "26249185#30_2":{"26249185#1_2","26249185#0_2"},
        "26249185#30_1":{"26249185#1_1","26249185#0_1"}},
        "intersection2": {
        "183049933#0_1":{},
        "758088375#0_1":{},
        "-25973410#1_1":{},
        "758088375#0_2":{},
        "-38361908#1_2":{},
        "-38361908#1_1":{}},
        "intersection3":{
        "E3_2": {},
        "E3_1":{},
        "-E1_1":{"-758088377#1_1"},
        "-E1_2":{"-758088377#1_2"},
        "-E4_2":{},
        "-E4_1":{},
        },
        "intersection4":{
        "22889927#0_1":{"758088377#2_1"},
        "-22889927#2_1":{},
        "-25576697#0_0":{}},
        "intersection5":{
        "E6_1":{"E5_1","130569446_1"},
        "E6_2":{"E5_2","130569446_2"},
        "E6_3":{"E5_2","130569446_2"},
        "-208691154#0_1":{"-208691154#1_1"},
        "E15_2":{},
        "E15_1":{},
        "E10_1":{"E9_1","1162834479#1_1"}
        }
    }
    cyclelength = 90
    throughput = 0
    delay_total = 0
    delay_sideroad = 0
    delay_mainroad = 0
    veh_total = 0
    veh_sideroad = 0
    veh_mainroad = 0
    TTT=0
    TD = 0
    flow = 0
    density = 0
    DS = 0
    waiting_times = 0
    queue_lengths = 0
    last_cycle_update = -90
    ################################
    # Launch SUMO
    traci.start([
        SUMO_BINARY,
        "-c", "../model/Configuration.sumocfg",
        "--quit-on-end",
        "--start",
        "--time-to-teleport", "-1",
        "--waiting-time-memory", "6000"
    ])
    
    # Load Vehicle Spawn Data
    df_veh_spawn = pd.read_csv("../model/Spawn_Vehicles.csv")
    df_veh_spawn = df_veh_spawn.rename(columns={"Unnamed: 0": "veh_ctr"})
    df_bus_spawn = pd.read_csv("../model/Spawn_Bus.csv")
    df_bus_spawn = df_bus_spawn.rename(columns={"Unnamed: 0": "veh_ctr"})
    
    # Initialize Max Pressure
    if CONTROL_MODE=="MAX_PRESSURE":
        for controller in signal_controllers:
            controller.current_gt_start = traci.simulation.getTime()
            
    # Recorder
    veh_routes = {}
    veh_classes = {}
    
    # Run Simulation
    veh_ctr = 0
    
    for current_time in SIMULATION_TIMES:
        #Update Vehicles
        df_current_status, df_hidden_vehicles = determine_current_state()
        #Initialize and Update Controllers
        if CONTROL_MODE == "SCOSCA":
            if step == last_cycle_update + cyclelength:
                queue_lengths = get_queue_lengths(lanes, up_stream_links, df_hidden_vehicles)
                cyclelength,greentimes = setup_scosca_control(queue_lengths,DS, step,
                                             adaptation_cycle, adaptation_green, green_thresh,
                                             adaptation_offset, offset_thresh,
                                             greentimes,cyclelength)
                last_cycle_update = step
            DS = calculate_degree_of_saturation_SCATS(greentimes, cyclelength, step, JUNCTION_IDS, lanes)
        elif CONTROL_MODE == "SCOSCAFAIRV1":
            if step == last_cycle_update + cyclelength:
                queue_lengths = get_queue_lengths(lanes, up_stream_links, df_hidden_vehicles)
                cyclelength, greentimes = setup_scoscafairv1_control(queue_lengths,DS,waiting_times, step,
                                             adaptation_cycle, adaptation_green, green_thresh,
                                             adaptation_offset, offset_thresh,
                                             greentimes,cyclelength,alpha)
                last_cycle_update = step
            waiting_times = get_waiting_times(cyclelength, lanes, up_stream_links, df_hidden_vehicles)
            DS = calculate_degree_of_saturation_SCATS(greentimes, cyclelength, step, JUNCTION_IDS, lanes)
        elif CONTROL_MODE == "SCOSCAFAIRV2":
            if step == last_cycle_update + cyclelength:
                queue_lengths = get_queue_lengths(lanes, up_stream_links, df_hidden_vehicles)
                cyclelength,greentimes = setup_scoscafairv2_control(queue_lengths,DS, step,
                                                 adaptation_cycle, adaptation_green, green_thresh,
                                                 adaptation_offset, offset_thresh, Changetime,
                                                 greentimes,cyclelength)
                last_cycle_update = step
            DS = calculate_degree_of_saturation_SCATS(greentimes, cyclelength, step, JUNCTION_IDS, lanes)
            Optimizer_Fairness(Changetime, Thresholdtime,greentimes)
        elif CONTROL_MODE=="MAX_PRESSURE":
            #Set Trafficlights for Max Pressure
            for controller in signal_controllers:
                controller.do_signal_logic()
                
        #Update Metrics
        if step >= 1800:
            throughput += get_throughput(lanes,step)
            flow += get_flow()
            TD += get_total_distance()
            delay_t,veh_t,delay_s,veh_s,delay_m,veh_m = get_average_delay_total()
            density += get_density()
            delay_total += delay_t
            delay_sideroad += delay_s
            delay_mainroad += delay_m
            veh_total += veh_t
            veh_sideroad += veh_s
            veh_mainroad += veh_m
            TTT += get_total_travel_time(step)
    
        #Spawn Vehicles
        for idx, row in df_veh_spawn[df_veh_spawn["Adjusted_Datetime"]==current_time].iterrows():
            for x in range(0, int(np.ceil(row["n_spawn"]))):
                veh_ctr += 1
                spawn_random_vehicle(veh_ctr, desired_route=str(row["route"]))
                
        #Spawn Buses
        for idx, row in df_bus_spawn[df_bus_spawn["Adjusted_Datetime"]==current_time].iterrows():
            veh_ctr += 1
            spawn_random_bus(veh_ctr, desired_route=str(row["route"]), stops=str(row["Stops"]))
            
        #Simulate for One Step
        for n in range(0,SIMULATION_STEPS_PER_SECOND):
            traci.simulationStep()
        if DEBUG_GUI:
            time.sleep(SIMULATION_WAIT_TIME)
    
        step += 1
        
    #Make Final Metric Calculations
    avg_delay = delay_total/veh_total
    avg_delay_sideroad = delay_sideroad/veh_sideroad
    avg_delay_mainroad = delay_mainroad/veh_mainroad
    avg_density = density/SIMULATION_DURATION
    avg_speed = TD/TTT
    gini = get_gini()
    max_delay = get_max_delay()
    
    #Print Metrics
    print(f"THROUGHPUT: {throughput}",flush=True)
    print(f"FLOW: {flow}",flush=True)
    print(f"AVG SPEED: {avg_speed}",flush=True)
    print(f"DENSITY: {avg_density}",flush=True)
    print(f"AVG DELAY: {avg_delay}",flush=True)
    print(f"AVG. DELAY SIDEROAD: {avg_delay_sideroad}",flush=True)
    print(f"AVG. DELAY MAINROAD: {avg_delay_mainroad}",flush=True)
    print(f"MAX DELAY: {max_delay}",flush=True)
    print(f"TOTAL TRAVEL TIME: {TTT}",flush=True)
    print(f"GINI TOTAL: {gini[0]}",flush=True)
    print(f"GINI SIDEROAD: {gini[1]}",flush=True)
    print(f"GINI MAINROAD: {gini[2]}",flush=True)
    
    # Close SUMO
    traci.close()
    
    #Return Metrics to Optimizer
    return (throughput,flow,avg_speed,avg_density,avg_delay,avg_delay_sideroad,
            avg_delay_mainroad,max_delay, TTT,gini[0],gini[1],gini[2])

"""
# #############################################################################
# ###### EXAMPLE HOW TO RUN THE SIMULATION MANUALLY ###########################
# #############################################################################

seed = 41
    # Parameters For SCOSCA
adaptation_cycle = 46.71
adaptation_green = 6.62
green_thresh = 0.79
adaptation_offset = 0.24
offset_thresh = 0.14
alpha = -1
Changetime = -1
Thresholdtime = -1
#     # Parameters For SCOSCA_1
# adaptation_cycle = 28.66
# adaptation_green = 14.99
# green_thresh = 2.41
# adaptation_offset = 0.32
# offset_thresh = 0.47
# alpha = 0.62
# Changetime = -1
# Thresholdtime = -1
#     # Parameters For SCOSCA_2
# adaptation_cycle = 39.28
# adaptation_green = 13.96
# green_thresh = 0.34
# adaptation_offset = 0.30
# offset_thresh = 0.55
# alpha = -1
# Changetime = 54.97
# Thresholdtime = 3.68
param_sets = (seed, adaptation_cycle, adaptation_green, green_thresh, adaptation_offset, offset_thresh,alpha, Changetime, Thresholdtime)
    # RUN SIMULATION
Simulation(param_sets)
"""