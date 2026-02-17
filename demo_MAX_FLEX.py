############## IMPORTS
import traci
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

from sumoITScontrol import Intersection
from sumoITScontrol.control.intersection_management import MaxPressure_Flex

############## PARAMETERS
simulation_parameters = {
    "sumo_config_file": "./demo_simulation_models/example_intersection_management/Configuration.sumocfg",
    "duration_sec": 86400-32400, # =24h-3h
    "time_step": 0.25, #s/step
    "start_time": datetime.strptime("09:00", "%H:%M"),
    "sumo_random_seed": 2
}

# SUMO
SUMO_BINARY = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo-gui.exe" # sumo.exe without GUI but faster # sumoBinary = "C:/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui.exe"
SUMO_CMD = [SUMO_BINARY, "-c", simulation_parameters["sumo_config_file"], "--start", "--quit-on-end", "--time-to-teleport", "-1", "--seed", str(simulation_parameters["sumo_random_seed"])]

# DEFINE INTERSECTIONS and CONTROLLER
intersection1 = Intersection(
    tl_id = "intersection1",
    phases = [0, 2, 4],
    links = {0:["921020465#1_3", "921020465#1_2", "921020464#0_1", "921020464#1_1", "38361907_3", "38361907_2", "-1164287131#1_3", "-1164287131#1_2"], 
             2:["-1169441386_2", "-1169441386_1", "-331752492#1_2", "-331752492#1_1", "-331752492#0_1", "-331752492#0_2"], 
             4:["-183419042#1_1", "26249185#30_1", "26249185#30_2", "26249185#1_1", "26249185#1_2"]},
    
)

intersection2 = Intersection(
    tl_id = "intersection2",
    phases = [0, 2, 4],
    # links = {0:["183049933#0_1", "-38361908#1_1"], 
    #           2:["-38361908#1_1", "-38361908#1_2"], 
    #           4:["-25973410#1_1", "758088375#0_1", "758088375#0_2"]},
    sensors = {0:["e2_183049933#0_1", "e2_-38361908#1_1"],
                2:["e2_-38361908#1_1", "e2_-38361908#1_2"],
                4:["e2_-25973410#1","e2_758088375#0_1", "e2_758088375#0_2"]}
)

intersection3 = Intersection(
    tl_id = "intersection3",
    phases = [0, 2, 4],
    links = {0:["E3_1", "-758088377#1_1", "-758088377#1_2", "-E1_1", "-E1_2"], 
             2:["E3_1", "E3_2"], 
             4:["-758088377#1_1", "-E1_1", "-E4_1", "-E4_2"]}
)

intersection4 = Intersection(
    tl_id = "intersection4",
    phases = [0, 2],
    links = {0:["22889927#0_1", "758088377#2_1", "-22889927#2_1"], 
             2:["-25576697#0_0"]}
)

intersection5 = Intersection(
    tl_id = "intersection5",
    phases = [0, 2, 4],
    links = {0:["E6_1", "E6_2", "E5_1", "130569446_1", "E15_1", "E15_2"], 
             2:["E15_2", "E6_3", "E5_2", "130569446_2"],
             4:["E10_1", "E9_1",  "1162834479#1_1", "-208691154#0_1", "-208691154#1_1"]},
)

max_pressure_params = {
    "T_A": 5,
    "T_L": 3,  # Yellow Time
    "G_T_MIN": 5,  # Min Greentime (used for Max. Pressure)
    "G_T_MAX": 50,  # Max Greentime (used for Max. Pressure)
    "measurement_period": int(1/0.25), #int(1 / simulation.time_step)
}

controller1 = MaxPressure_Flex(
    params = max_pressure_params,
    intersection = intersection1
)

controller2 = MaxPressure_Flex(
    params = max_pressure_params,
    intersection = intersection2
)

controller3 = MaxPressure_Flex(
    params = max_pressure_params,
    intersection = intersection3
)

controller4 = MaxPressure_Flex(
    params = max_pressure_params,
    intersection = intersection4
)

controller5 = MaxPressure_Flex(
    params = max_pressure_params,
    intersection = intersection5
)

signal_controllers = [controller1, controller2, controller3, controller4, controller5]




######## SIMULATION
# Start Sumo
traci.start(SUMO_CMD)
# Initialize
# Execute Simulation
for simulation_timestep in range(0, int(simulation_parameters["duration_sec"]/simulation_parameters["time_step"])):
    # run one step
    traci.simulationStep()
    current_time = traci.simulation.getCurrentTime()
    # execute control
    for controller in signal_controllers:
        controller.execute_control(current_time)
# Stop Sumo
traci.close()


