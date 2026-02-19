"""
This demo script showcases the application of the HERO (Hierarchical Ramp Metering Coordination) algorithm for adaptive ramp metering control in a SUMO simulation environment.
The script defines a coordination group of three ramp meters, each with its own set of mainline and queue sensors. 
The HERO controller dynamically adjusts the metering rates of the ramp meters based on real-time traffic conditions, 
aiming to optimize traffic flow on the mainline while managing queue lengths on the ramps.
"""


############## IMPORTS
import numpy as np
import traci
from datetime import datetime
import warnings

warnings.filterwarnings("ignore")

from sumoITScontrol import RampMeter, RampMeterCoordinationGroup
from sumoITScontrol.control.ramp_metering import ALINEA, HERO

############## PARAMETERS
simulation_parameters = {
    "sumo_config_file": "./demo_simulation_models/example_ramp_metering/Configuration_4.sumocfg",
    "duration_sec": 4200,  # =3h
    "time_step": 0.5,  # s/step
    "start_time": datetime.strptime("07:50", "%H:%M"),
    "sumo_random_seed": 2,
}

# SUMO
SUMO_BINARY = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo-gui.exe"  # sumo.exe without GUI but faster # sumoBinary = "C:/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui.exe"
SUMO_CMD = [
    SUMO_BINARY,
    "-c",
    simulation_parameters["sumo_config_file"],
    "--start",
    "--quit-on-end",
    "--time-to-teleport",
    "-1",
    "--seed",
    str(simulation_parameters["sumo_random_seed"]),
]

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
    ramp_meter_ids=["J12", "J11", "J0"],
)

controller = HERO(
    params={
        "hero_cycle_duration": 60,  # similar to ALINEA cycle duration
        "queue_activation_threshold_m": 15.0,  # master queue trigger
        "queue_release_threshold_m": 2.5,  # dissolve cluster
        "min_queue_setpoint_m": 5.0,  # for slaves
        "anticipation_factor": 1.0,  # factor to obtain nonconservative prediction of demand to come in next control period
        "avg_vehicle_spacing": 7.5,  # average vehicle spacing to convert meters to vehicles and vice versa, from queue length measurements
    },
    coordination_group=ramp_meter_group,
    alinea_controllers={
        "J12": ALINEA(
            params={
                "target_occupancy": 10,
                "K_P": 30,
                "K_I": 0,
                "cycle_duration": 60,
                "measurement_period": int(
                    60 / 0.5
                ),  # int(cycle_duration / simulation.time_step)
                "min_rate": 5,
                "max_rate": 100,
            },
            ramp_meter=ramp_meter_group.ramp_meters[0],
        ),
        "J11": ALINEA(
            params={
                "target_occupancy": 10,
                "K_P": 30,
                "K_I": 0,
                "cycle_duration": 60,
                "measurement_period": int(
                    60 / 0.5
                ),  # int(cycle_duration / simulation.time_step)
                "min_rate": 5,
                "max_rate": 100,
            },
            ramp_meter=ramp_meter_group.ramp_meters[1],
        ),
        "J0": ALINEA(
            params={
                "target_occupancy": 10,
                "K_P": 30,
                "K_I": 0,
                "cycle_duration": 60,
                "measurement_period": int(
                    60 / 0.5
                ),  # int(cycle_duration / simulation.time_step)
                "min_rate": 5,
                "max_rate": 100,
            },
            ramp_meter=ramp_meter_group.ramp_meters[2],
        ),
    },
)


######## SIMULATION
lightsA = []
lightsB = []
lightsC = []
# Start Sumo
traci.start(SUMO_CMD)
# Initialize
# Execute Simulation
for simulation_timestep in range(
    0, int(simulation_parameters["duration_sec"] / simulation_parameters["time_step"])
):
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


# ######## VISUALIZATION
import matplotlib.pyplot as plt

plt.figure(figsize=(8, 2.75))
plt.subplot(1, 3, 1)
for alinea_controller_name in controller.alinea:
    alinea_controller = controller.alinea[alinea_controller_name]
    plt.step(
        np.asarray(alinea_controller.measurement_data["metering_rate"])[:, 0],
        np.asarray(alinea_controller.measurement_data["metering_rate"])[:, 1],
        label=alinea_controller_name,
    )
plt.legend()
plt.xlabel("Simulation Time [s]")
plt.ylabel("Metering Rate [%]")

plt.subplot(1, 3, 2)
for alinea_controller_name in controller.alinea:
    alinea_controller = controller.alinea[alinea_controller_name]
    plt.step(
        np.asarray(alinea_controller.measurement_data["metering_occupancy"])[:, 0],
        np.asarray(alinea_controller.measurement_data["metering_occupancy"])[:, 1],
        label=alinea_controller_name,
    )
plt.legend()
plt.xlabel("Simulation Time [s]")
plt.ylabel("Mainline State (Occupancy [%])")

plt.subplot(1, 3, 3)
for alinea_controller_name in controller.alinea:
    alinea_controller = controller.alinea[alinea_controller_name]
    plt.step(
        np.asarray(alinea_controller.measurement_data["queue_length_m"])[:, 0],
        np.asarray(alinea_controller.measurement_data["queue_length_m"])[:, 1],
        label=alinea_controller_name,
    )
plt.legend()
plt.xlabel("Simulation Time [s]")
plt.ylabel("Queue State (Lenght [m])")

plt.tight_layout()
plt.show()



plt.figure(figsize=(8, 2))

plt.subplot(1, 3, 1)
plt.title("J0")
plt.plot(lightsA, label="J0")
plt.xlabel("Simulation Time [s]")
plt.xlim(2500,6000)
plt.subplot(1, 3, 2)
plt.title("J11")
plt.plot(lightsB, label="J11")
plt.xlabel("Simulation Time [s]")
plt.xlim(2500,6000)
plt.subplot(1, 3, 3)
plt.title("J12")
plt.plot(lightsC, label="J12")
plt.xlabel("Simulation Time [s]")
plt.xlim(2500,6000)
plt.tight_layout()

plt.show()




plt.figure(figsize=(8, 2))
time = controller.log["time"]
ramp_ids = list(controller.log["ramps"].keys())[:3]  
ramp_ids.reverse()
for i, ramp_id in enumerate(ramp_ids, 1):
    data = controller.log["ramps"][ramp_id]
   
    plt.subplot(1, 3, i)
    plt.step(time, np.zeros(len(time)), where="post")
    plt.title(f"Role at {ramp_id}")
    plt.xlabel("Time [s]")
    plt.ylim(0,20)
    for t, role in zip(time, data["role"]):
        if role == "MASTER":
            plt.axvline(t, color="r", alpha=0.1)
        elif role == "SLAVE":
            plt.axvline(t, color="b", alpha=0.05)
plt.tight_layout()
plt.show()