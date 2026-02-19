"""
This demo script showcases the application of the SCOSCA (Self-COordinating SCOOT and SCATS) algorithm for adaptive traffic signal control in a SUMO simulation environment. 
The script defines a network of five intersections, each with its own set of traffic signal phases and associated links. 
The SCOSCA controller dynamically adjusts the green times of the traffic signals based on real-time traffic conditions, aiming to optimize traffic flow across the network.
"""

############## IMPORTS
import traci
from datetime import datetime
import warnings

warnings.filterwarnings("ignore")

from sumoITScontrol import Intersection, IntersectionGroup
from sumoITScontrol.control.intersection_management.ScootScats import ScootScats

############## PARAMETERS
simulation_parameters = {
    "sumo_config_file": "./demo_simulation_models/example_intersection_management/Configuration.sumocfg",
    "duration_sec": 3600,#86400 - 32400,  # 24h - 9h
    "time_step": 0.25,
    "start_time": datetime.strptime("09:00", "%H:%M"),
    "sumo_random_seed": 2,
}

# SUMO
SUMO_BINARY = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo-gui.exe"
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

# DEFINE INTERSECTIONS and CONTROLLER
intersection1 = Intersection(
    tl_id="intersection1",
    phases=[0, 2, 4],
    links={
        0: [
            "921020465#1_3",
            "921020465#1_2",
            "921020464#0_1",
            "921020464#1_1",
            "38361907_3",
            "38361907_2",
            "-1164287131#1_3",
            "-1164287131#1_2",
        ],
        2: [
            "-1169441386_2",
            "-1169441386_1",
            "-331752492#1_2",
            "-331752492#1_1",
            "-331752492#0_1",
            "-331752492#0_2",
        ],
        4: [
            "-183419042#1_1",
            "26249185#30_1",
            "26249185#30_2",
            "26249185#1_1",
            "26249185#1_2",
        ],
    },
    green_states=["GGrrGrr", "rrGGrrr", "rrrrrGG"],
    yellow_states=["yyrryrr", "rryyrrr", "rrrrryy"],
)
intersection2 = Intersection(
    tl_id="intersection2",
    phases=[0, 2, 4],
    links={
        0: ["183049933#0_1", "-38361908#1_1"],
        2: ["-38361908#1_1", "-38361908#1_2"],
        4: ["-25973410#1_1", "758088375#0_1", "758088375#0_2"],
    },
    green_states=["GGrrrGrr", "GGGrrrrr", "rrrGGrGG"],
    yellow_states=["yyrrryrr", "yyyrrrrr", "rrryyryy"],
)
intersection3 = Intersection(
    tl_id="intersection3",
    phases=[0, 2, 4],
    links={
        0: ["E3_1", "-758088377#1_1", "-758088377#1_2", "-E1_1", "-E1_2"],
        2: ["E3_1", "E3_2"],
        4: ["-758088377#1_1", "-E1_1", "-E4_1", "-E4_2"],
    },
    green_states=["GGGrrrr", "rrGGrrr", "GrrrGGG"],
    yellow_states=["yyyrrrr", "rryyrrr", "yrrryyy"],
)
intersection4 = Intersection(
    tl_id="intersection4",
    phases=[0, 2],
    links={0: ["22889927#0_1", "758088377#2_1", "-22889927#2_1"], 2: ["-25576697#0_0"]},
    green_states=["GgrrGG", "rrGGGr"],
    yellow_states=["yyrryy", "rryyyr"],
)
intersection5 = Intersection(
    tl_id="intersection5",
    phases=[0, 2, 4],
    links={
        0: ["E6_1", "E6_2", "E5_1", "130569446_1", "E15_1", "E15_2"],
        2: ["E15_2", "E6_3", "E5_2", "130569446_2"],
        4: ["E10_1", "E9_1", "1162834479#1_1", "-208691154#0_1", "-208691154#1_1"],
    },
    green_states=["GGrrrrGGrrrr", "rrGrrrrrGrrr", "rrrGGgrrrGGg"],
    yellow_states=["yyrrrryyrrrr", "rryrrrrryrrr", "rrryyyrrryyy"],
)
districts = {
    "front": ["intersection1", "intersection2"],
    "middle": ["intersection3", "intersection4"],
    "back": ["intersection5"],
}
critical_district_order = {
    "front": [
        "intersection1",
        "intersection2",
        "intersection3",
        "intersection4",
        "intersection5",
    ],
    "middle": [
        "intersection3",
        "intersection2",
        "intersection4",
        "intersection1",
        "intersection5",
    ],
    "back": [
        "intersection5",
        "intersection4",
        "intersection3",
        "intersection2",
        "intersection1",
    ],
}
connection_between_intersections = {
    "intersection1": ["183049934_1", "183049933#0_1", "1164287131#0_1"],  # To Int 2
    "intersection2": ["38361908#1_1", "E3_1"],  # To Int 3
    "intersection3": [
        "E1_1",
        "758088377#1_1",
        "758088377#2_1",
        "22889927#0_1",
    ],  # To Int 4
    "intersection4": [
        "22889927#2_1",
        "22889927#3_1",
        "22889927#4_1",
        "387296014#0_1",
        "387296014#1_1",
        "696225646#1_1",
        "696225646#2_1",
        "696225646#3_1",
        "130569446_1",
        "E5_1",
        "E6_1",
    ],  # To Int 5
}
intersection_group = IntersectionGroup(
    intersections=[
        intersection1,
        intersection2,
        intersection3,
        intersection4,
        intersection5,
    ],
    districts=districts,
    critical_district_order=critical_district_order,
    connection_between_intersections=connection_between_intersections,
)
initial_greentimes = {
    "intersection1": [30, 30, 21],
    "intersection2": [30, 30, 21],
    "intersection3": [30, 30, 21],
    "intersection4": [40, 30],
    "intersection5": [30, 30, 21],
}
scosca_params = {
    "adaptation_cycle": 30,
    "adaptation_green": 10,
    "green_thresh": 2,
    "adaptation_offset": 1,
    "offset_thresh": 0.5,
    "min_cycle_length": 50,
    "max_cycle_length": 180,
    "ds_upper_val": 0.925,
    "ds_lower_val": 0.875,
    "measurement_period": int(1 / 0.25),  # 1 / simulation_step_size
    "travel_time_adjustments": {
        "intersection1": ["183049934_1", 2],
        "intersection3": ["E1_1", 3],
        "intersection4": ["22889927#3_1", 9],
    },
    "intersection_offset_rules": {
        "intersection2": {
            "base_offset_from": None,
            "travel_time_from": "intersection2",
        },
        "intersection4": {
            "base_offset_from": None,
            "travel_time_from": "intersection3",
        },
        "intersection1": {
            "base_offset_from": "intersection2",
            "travel_time_from": "intersection1",
        },
        "intersection3": {
            "base_offset_from": "intersection4",
            "travel_time_from": "intersection4",
        },
        "default": {
            "base_offset_from": "intersection4",
            "travel_time_from": "intersection4",
        },
    },
}
controller = ScootScats(
    scosca_params, intersection_group, initial_greentimes, initial_cycle_length=120
)


######## SIMULATION
# Start Sumo
traci.start(SUMO_CMD)
# Initialize
intersection_group._init_lane_lengths()
# Execute Simulation
for simulation_timestep in range(
    0, int(simulation_parameters["duration_sec"] / simulation_parameters["time_step"])
):
    # run one step
    traci.simulationStep()
    current_time = traci.simulation.getCurrentTime()
    # execute control
    controller.execute_control(current_time)
# Stop Sumo
traci.close()


######## VISUALIZATION
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timedelta
import matplotlib.dates as mdates
from collections import defaultdict

def generate_spat_figure(controller, intersection_name,
                         xlim=(20502.750402160993, 20502.75300752443)):
    data = controller.measurement_data["history_greentimes"]
    # Extract timestamps
    timestamps_ms = [d[0] for d in data]
    # Extract green times only for selected intersection
    green_data = [d[1][intersection_name] for d in data]
    # Determine number of phases dynamically
    num_phases = len(green_data[0])
    unique_phases = list(range(1, num_phases + 1))
    # Convert timestamps to datetime
    base_time = datetime(2026, 2, 18, 9, 0, 0)
    timestamps_dt = [base_time + timedelta(milliseconds=ts)
                     for ts in timestamps_ms]
    timestamps_num = mdates.date2num(timestamps_dt)
    # --- Build phase intervals ---
    phase_intervals = defaultdict(list)
    for i in range(len(data)):
        start_time_num = timestamps_num[i]
        green_times = green_data[i]
        curr_time = start_time_num
        for phase_index, green_sec in enumerate(green_times):
            phase = phase_index + 1
            end_time = curr_time + green_sec / (24 * 3600)
            phase_intervals[phase].append((curr_time, end_time))
            curr_time = end_time
    # --- Segment builder (red/yellow/green) ---
    def get_segments_for_phase(phase):
        intervals = phase_intervals[phase]
        segments = []
        yellow_duration = 3 / (24 * 3600)
        min_time_num = timestamps_num[0]
        max_time_num = timestamps_num[-1] + sum(green_data[-1]) / (24 * 3600)
        curr_time = min_time_num
        for start, end in intervals:
            # Red before green
            if curr_time < start:
                segments.append(('red', curr_time, start))
            # Yellow before green
            y_start = max(curr_time, start - yellow_duration)
            y_end = start
            if y_end > y_start:
                segments.append(('yellow', y_start, y_end))
            # Green
            segments.append(('green', start, end))
            curr_time = max(curr_time, end)
        # Final red
        if curr_time < max_time_num:
            segments.append(('red', curr_time, max_time_num))
        return segments
    # --- Plot ---
    fig, ax = plt.subplots(figsize=(8, 2.5))
    for y_pos, phase in enumerate(unique_phases):
        segments = get_segments_for_phase(phase)
        for color, s, e in segments:
            if e > s:
                ax.barh(y_pos, e - s, left=s, height=0.6,
                        color=color, edgecolor='black', linewidth=0.5)
        ax.text(timestamps_num[0] - 0.0003, y_pos,
                f'Phase {phase}', va='center', fontweight='bold')
    # Axis formatting
    max_time = timestamps_num[-1] + sum(green_data[-1]) / (24 * 3600)
    ax.set_xlim(timestamps_num[0], max_time)
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    ax.xaxis.set_major_locator(mdates.SecondLocator(interval=30))
    plt.setp(ax.xaxis.get_majorticklabels(), rotation=45)
    ax.set_yticks(range(len(unique_phases)))
    ax.set_yticklabels([f'Phase {p}' for p in unique_phases])
    ax.set_ylim(-0.5, len(unique_phases) - 0.5)
    if xlim is not None:
        ax.set_xlim(*xlim)
    ax.set_title(f'SPAT Plan - {intersection_name}')
    fig.autofmt_xdate()
    plt.tight_layout()
    plt.show()

def generate_schedule_figure(controller):
    data = controller.measurement_data["history_greentimes"]
    # Extract timestamps
    timestamps_ms = [d[0] for d in data]
    # Convert timestamps to datetime
    base_time = datetime(2026, 2, 18, 9, 0, 0)
    timestamps_dt = [base_time + timedelta(milliseconds=ts) for ts in timestamps_ms]
    # Extract intersection names from first entry
    intersection_names = list(data[0][1].keys())
    n = len(intersection_names)
    fig, axes = plt.subplots(n, 1, figsize=(8, 6), sharex=True)
    if n == 1:
        axes = [axes]
    for i, intersection in enumerate(intersection_names):
        # Extract green durations for this intersection
        green_durations = np.array([
            d[1][intersection] for d in data
        ])
        # --- Step duplication ---
        step_times = []
        step_values = []
        for j in range(len(timestamps_dt) - 1):
            step_times.append(timestamps_dt[j])
            step_values.append(green_durations[j])
            step_times.append(timestamps_dt[j + 1])
            step_values.append(green_durations[j])
        step_times.append(timestamps_dt[-1])
        step_values.append(green_durations[-1])
        step_values = np.array(step_values).T
        # --- Stackplot ---
        num_phases = green_durations.shape[1]
        phase_labels = [f"P{p+1}" for p in range(num_phases)]
        colors = ['green', 'blue', 'orange', 'purple', 'cyan'][:num_phases]
        axes[i].stackplot(
            step_times,
            step_values,
            labels=phase_labels,
            colors=colors,
            alpha=0.7
        )
        axes[i].set_ylabel(f"{intersection}\nGreen (s)")
        axes[i].set_ylim(0, max(step_values.sum(axis=0)) * 1.1)
        axes[i].legend(loc='upper left', fontsize=7)
        # Only bottom subplot gets x-axis labels
        if i != n - 1:
            axes[i].tick_params(labelbottom=False)
        else:
            axes[i].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
            axes[i].xaxis.set_major_locator(mdates.SecondLocator(interval=200))
            plt.setp(axes[i].xaxis.get_majorticklabels(), rotation=45)
            axes[i].set_xlabel("Time")
    plt.tight_layout()
    plt.show()

generate_spat_figure(controller, "intersection1", xlim=(20502.776855004264, 20502.7794603677))
generate_spat_figure(controller, "intersection2", xlim=(20502.776855004264, 20502.7794603677))
generate_spat_figure(controller, "intersection3", xlim=(20502.776855004264, 20502.7794603677))
generate_spat_figure(controller, "intersection4", xlim=(20502.776855004264, 20502.7794603677))
generate_spat_figure(controller, "intersection5", xlim=(20502.776855004264, 20502.7794603677))

generate_schedule_figure(controller)