############## IMPORTS
import numpy as np
import traci
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

from sumoITScontrol import RampMeter, RampMeterCoordinationGroup
from sumoITScontrol.control.ramp_metering import METALINE

############## PARAMETERS
simulation_parameters = {
    "sumo_config_file": "./demo_simulation_models/example_ramp_metering/Configuration_4.sumocfg",
    "duration_sec": 4200, # =3h
    "time_step": 0.5, #s/step
    "start_time": datetime.strptime("07:50", "%H:%M"),
    "sumo_random_seed": 2
}

# SUMO
SUMO_BINARY = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo-gui.exe" # sumo.exe without GUI but faster # sumoBinary = "C:/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui.exe"
SUMO_CMD = [SUMO_BINARY, "-c", simulation_parameters["sumo_config_file"], "--start", "--quit-on-end", "--time-to-teleport", "-1", "--seed", str(simulation_parameters["sumo_random_seed"])]

# DEFINE METER and CONTROLLER
ramp_meter_group = RampMeterCoordinationGroup(
    ramp_meters_ordered=[
        RampMeter(
            tl_id="J12",
            mainline_sensors=["e1_13", "e1_14"], 
            queue_sensors=["e2_1", "e2_2"],
            smoothening_factor=0.1,
            saturation_flow_veh_per_sec=0.5,
        ),
        RampMeter(
            tl_id="J11",
            mainline_sensors=["e1_2", "e1_3"], 
            queue_sensors=["e2_3"],
            smoothening_factor=0.1,
            saturation_flow_veh_per_sec=0.5,
        ),
        RampMeter(
            tl_id="J0",
            mainline_sensors=["e2_5", "e2_4"], 
            queue_sensors=["e2_0"],
            smoothening_factor=0.1,
            saturation_flow_veh_per_sec=0.5,
        ),
    ],
    ramp_meter_ids=["J12", "J11", "J0"])    
        
controller = METALINE(
    params={
        "cycle_duration": 60,  # control cycle
        "measurement_period": int(60/0.5), #int(cycle_duration / simulation.time_step)
        "min_rate": 5,
        "max_rate": 100
    },
    coordination_group=ramp_meter_group,
    target_occupancies=[10, 10, 10],
    # Interaction gain matrix (3x3)
    K_P = np.array([
        [30, -5, 0],   # ramp 1 influenced negatively by ramp 2
        [-3, 25, -2],  # ramp 2 influenced by neighbors
        [0, -4, 20]
    ]),
    # Optional integral gain matrix
    K_I = np.zeros(shape=(3,3)),
)




lightsA = []
lightsB = []
lightsC = []
######## SIMULATION
# Start Sumo
traci.start(SUMO_CMD)
# Initialize
# Execute Simulation
for simulation_timestep in range(0, int(simulation_parameters["duration_sec"]/simulation_parameters["time_step"])):
    # run one step
    traci.simulationStep()
    current_time = traci.simulation.getCurrentTime()
    lightsA.append(traci.trafficlight.getRedYellowGreenState("J0"))
    lightsB.append(traci.trafficlight.getRedYellowGreenState("J11"))
    lightsC.append(traci.trafficlight.getRedYellowGreenState("J12"))
    # execute control
    controller.execute_control(current_time)
# Stop Sumo
traci.close()

lightsA = [1 if str(x).startswith("G") else 0 for x in lightsA]
lightsB = [1 if str(x).startswith("G") else 0 for x in lightsB]
lightsC = [1 if str(x).startswith("G") else 0 for x in lightsC]



# ######## VISUALIZATION
import matplotlib.pyplot as plt

plt.figure(figsize=(8,3))
plt.subplot(1,3,1)
for ramp_name in controller.measurement_data.keys():
    plt.step(np.asarray(controller.measurement_data[ramp_name]["metering_rate"])[:,0],
              np.asarray(controller.measurement_data[ramp_name]["metering_rate"])[:,1],
              label=ramp_name)
plt.legend()
plt.xlabel("Simulation Time [s]")
plt.ylabel("Metering Rate [%]")

plt.subplot(1,3,2)
for ramp_name in controller.measurement_data.keys():
    plt.step(np.asarray(controller.measurement_data[ramp_name]["occupancy"])[:,0],
              np.asarray(controller.measurement_data[ramp_name]["occupancy"])[:,1],
              label=ramp_name)
plt.legend()
plt.xlabel("Simulation Time [s]")
plt.ylabel("Mainline State (Occupancy [%])")

plt.subplot(1,3,3)
for ramp_name in controller.measurement_data.keys():
    plt.step(np.asarray(controller.measurement_data[ramp_name]["queue_length_m"])[:,0],
              np.asarray(controller.measurement_data[ramp_name]["queue_length_m"])[:,1],
              label=ramp_name)
plt.legend()
plt.xlabel("Simulation Time [s]")
plt.ylabel("Queue State (Lenght [m])")

plt.tight_layout()
plt.show()




import matplotlib.pyplot as plt

plt.figure(figsize=(8,3))

plt.subplot(1,3,1)
plt.title("J0")
plt.plot(lightsA, label="J0")
plt.subplot(1,3,2)
plt.title("J11")
plt.plot(lightsB, label="J11")
plt.subplot(1,3,3)
plt.title("J12")
plt.plot(lightsC, label="J12")
