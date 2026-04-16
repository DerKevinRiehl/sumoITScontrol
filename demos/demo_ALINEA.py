"""
This demo script showcases the application of the ALINEA algorithm for adaptive ramp metering control in a SUMO simulation environment.
The script defines a single ramp meter with its own set of mainline and queue sensors.
"""

############## IMPORTS
import numpy as np
import traci
from datetime import datetime
import warnings

warnings.filterwarnings("ignore")

from sumoITScontrol import RampMeter
from sumoITScontrol.control.ramp_metering import ALINEA

############## PARAMETERS
simulation_parameters = {
    "sumo_config_file": "./demo_simulation_models/example_ramp_metering/Configuration_1.sumocfg",
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

ramp_meter = RampMeter(
    tl_id="J0",
    mainline_sensors=["e2_5", "e2_4"],
    queue_sensors=["e2_0"],
)

controller = ALINEA(
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
    ramp_meter=ramp_meter,
)


lights = []
######## SIMULATION
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
    lights.append(traci.trafficlight.getRedYellowGreenState("J0"))
    # execute control
    controller.execute_control(current_time)
# Stop Sumo
traci.close()


######## VISUALIZATION
import matplotlib.pyplot as plt

plt.figure(figsize=(8, 2.75))
plt.subplot(1, 3, 1)
plt.xlabel("Simulation Time [s]")
plt.ylabel("Metering Rate [%]")
plt.step(
    np.asarray(controller.measurement_data["metering_rate"])[:, 0],
    np.asarray(controller.measurement_data["metering_rate"])[:, 1],
)

plt.subplot(1, 3, 2)
plt.step(
    np.asarray(controller.measurement_data["metering_occupancy"])[:, 0],
    np.asarray(controller.measurement_data["metering_occupancy"])[:, 1],
)
plt.xlabel("Simulation Time [s]")
plt.ylabel("Mainline State (Occupancy [%])")

plt.subplot(1, 3, 3)
plt.step(
    np.asarray(controller.measurement_data["queue_length_m"])[:, 0],
    np.asarray(controller.measurement_data["queue_length_m"])[:, 1],
)
plt.xlabel("Simulation Time [s]")
plt.ylabel("Queue State (Lenght [m])")

plt.tight_layout()
plt.show()


plt.figure(figsize=(8, 2))
plt.title("J0")
plt.plot(lights, label="J0")
plt.xlabel("Simulation Time [s]")
plt.ylabel("LSA Phases")
plt.tight_layout()

