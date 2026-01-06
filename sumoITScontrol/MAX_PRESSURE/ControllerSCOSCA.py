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
    This script contains the implementation of the SCOOTS/SCATS (SCOSCA) traffic
    light controller.
"""




# #############################################################################
# ###### IMPORTS ##############################################################
# #############################################################################
import traci
from traci import tc




# #############################################################################
# ###### GLOBAL VARIABLES #####################################################
# #############################################################################

    # Tracks how often traffic lights get updated
update_counter = 0
    # Tracks the previous cycle length (used for proportional adjustments)
prev_cycle_length = {"intersection1": 81,
                     "intersection2": 81,
                     "intersection3": 81,
                     "intersection4": 84,
                     "intersection5": 81}
    # Offset timings between junctions (used for green wave coordination)
offsets = {}
    # Caching and helper structures
most_congested_edge = {}
most_congested_lane = {}
downstream_junction_map = {}
estimated_travel_time = {}
    # Save Green and Yellow states
green_states = {
    "intersection1": ["GGrrGrr", "rrGGrrr", "rrrrrGG"],
    "intersection2": ["GGrrrGrr", "GGGrrrrr", "rrrGGrGG"],
    "intersection3": ["GGGrrrr", "rrGGrrr", "GrrrGGG"],
    "intersection4": ["GgrrGG", "rrGGGr"],
    "intersection5": ["GGrrrrGGrrrr", "rrGrrrrrGrrr", "rrrGGgrrrGGg"]
}
yellow_states = {
    "intersection1": ["yyrryrr", "rryyrrr", "rrrrryy"],
    "intersection2": ["yyrrryrr", "yyyrrrrr", "rrryyryy"],
    "intersection3": ["yyyrrrr", "rryyrrr", "yrrryyy"],
    "intersection4": ["yyrryy", "rryyyr"],
    "intersection5": ["yyrrrryyrrrr", "rryrrrrryrrr", "rrryyyrrryyy"]
}
    # Save phases per junction 
phases_per_junction = {"intersection1":
            {0:["921020465#1_3", "921020465#1_2", "921020465#1_2", "921020464#0_1", "921020464#1_1", "38361907_3", "38361907_2", "-1164287131#1_3", "-1164287131#1_2"], 
             2:["-1169441386_2", "-1169441386_1", "-331752492#1_2", "-331752492#1_1", "-331752492#0_1", "-331752492#0_2"], 
             4:["-183419042#1_1", "26249185#30_1", "26249185#30_2", "26249185#1_1", "26249185#1_2"]},
                       "intersection2":
            {0:["183049933#0_1", "-38361908#1_1"], 
             2:["-38361908#1_1", "-38361908#1_2"], 
             4:["-25973410#1_1", "758088375#0_1", "758088375#0_2"]},
                       "intersection3":
            {0:["E3_1", "-758088377#1_1", "-758088377#1_2", "-E1_1", "-E1_2"], 
             2:["E3_1", "E3_2"], 
             4:["-758088377#1_1", "-E1_1", "-E4_1", "-E4_2"]},
                        "intersection4":
            {0:["22889927#0_1", "758088377#2_1", "-22889927#2_1"], 
             2:["-25576697#0_1"]},
                        "intersection5":
            {0:["E6_1", "E6_2", "E5_1", "130569446_1", "E15_1", "E15_2"], 
             2:["E15_2", "E6_3", "E5_2", "130569446_2"],
             4:["E10_1", "E9_1",  "1162834479#1_1", "-208691154#0_1", "-208691154#1_1"]}}
    # Save Connection lanes between junctions
connection = {"intersection1": ["183049934_1","183049933#0_1","1164287131#0_1"], #To Int 2
              "intersection2": ["38361908#1_1","E3_1"], #To Int 3
              "intersection3": ["E1_1","758088377#1_1","758088377#2_1","22889927#0_1"], # To Int 4
              "intersection4": ["22889927#2_1","22889927#3_1","22889927#4_1","387296014#0_1","387296014#1_1","696225646#1_1","696225646#2_1","696225646#3_1","130569446_1","E5_1","E6_1"] #To Int 5
             }




# #############################################################################
# ## CYCLE LENGTH OPTIMIZER
# #############################################################################
def optimize_cycle_length(degree_of_sat, cycle_length, adaptation_factor):
    """
    1. Optimize total cycle length of the network
    """
    max_ds_in_network = max(max(lane.values()) for lane in degree_of_sat.values())
    if max_ds_in_network >= 0.925:
        # Increase cycle length if highly saturated (SCOOT principle)
        new_length = cycle_length + (max_ds_in_network - 0.925) * adaptation_factor
        return min(int(new_length), 120)
    elif 0 < max_ds_in_network < 0.875:
        # Decrease if network is underused
        new_length = cycle_length - (0.875 - max_ds_in_network) * adaptation_factor
        return max(int(new_length), 50)
    else:
        # If DS is in a good range, keep the current cycle
        return cycle_length
    
# #############################################################################
# ## GREEN PHASE OPTIMIZER
# #############################################################################
def optimize_green_phases(queue_lengths, degree_of_sat, cycle_length,greentimes, adaptation_factor, threshold):
    """
    2. Optimize green phase duration per junction
    """
    global prev_cycle_length
    for junction in greentimes.keys():
        greens = greentimes[junction]
        effective_cycle = cycle_length - 3*(len(greens))  # Subtract yellow phases
        # Get the lane with the highest degree of saturation
        max_lane = max(degree_of_sat[junction], key=degree_of_sat[junction].get)
        max_queue = queue_lengths[junction][max_lane]
        max_ds = degree_of_sat[junction][max_lane]
        if max_queue > threshold:
            # Identify all phases where max_lane appears
            phases_with_max_lane = [
                p for p, lane_list in phases_per_junction[junction].items()
                if max_lane in lane_list
            ]
            # Determine the phase of max_lane
            if len(phases_with_max_lane) == 1:
                max_phase = phases_with_max_lane[0]//2
            else:
                max_phase = 0  # fallback if lane is used in multiple phases
            # Collect all lanes in those phases (to exclude them)
            excluded_lanes = set()
            for p in phases_with_max_lane:
                excluded_lanes.update(phases_per_junction[junction][p])
            # Get DS per other phase (not containing max_lane)
            phase_ds_list = []
            for phase in range(len(greens)):
                if phase == max_phase:
                    continue
                lanes = phases_per_junction[junction][phase * 2]
                phase_lanes = [l for l in lanes if l not in excluded_lanes]
                if phase_lanes:
                    ds_val = max([degree_of_sat[junction].get(l, 0) for l in phase_lanes])
                else:
                    ds_val = 0
                phase_ds_list.append((phase, ds_val))
            #Sort DS
            phase_ds_sorted = sorted(phase_ds_list, key=lambda x: x[1], reverse=True)
            second_phase = phase_ds_sorted[0][0]
            second_ds = phase_ds_sorted[0][1]
            third_phase = phase_ds_sorted[1][0] if len(phase_ds_sorted) > 1 else None
            third_ds = phase_ds_sorted[1][1] if len(phase_ds_sorted) > 1 else second_ds
            # Compute DS differences
            ds_diff_max = max(0, max_ds - third_ds)
            ds_diff_second = max(0, second_ds - third_ds)
            greentimes[junction][max_phase] = int(min(3 * effective_cycle / 4,
                                                    greentimes[junction][max_phase] + ds_diff_max * adaptation_factor))
            # Assign the remaining green time
            remaining_time = effective_cycle - greentimes[junction][max_phase]           
            if len(greens) == 2:
                # Only one other phase — assign remaining time to it
                greentimes[junction][second_phase] = remaining_time
            elif len(greens) == 3:
                # Determine saturation levels for the two remaining phases
                greentimes[junction][second_phase] = int(min(2*remaining_time/3, greentimes[junction][second_phase]+ds_diff_second*adaptation_factor))
                greentimes[junction][third_phase] = remaining_time - greentimes[junction][second_phase]               
        else:
            # Only scale with cycle length
                scaled = [int(g * effective_cycle / prev_cycle_length[junction]) for g in greens]
                # Normalize to match effective_cycle exactly
                diff = effective_cycle - sum(scaled)
                scaled[0] += diff  # Adjust first to balance
                greentimes[junction] = scaled
        prev_cycle_length[junction] = effective_cycle
    return greentimes

# #############################################################################
# ## OFFSET OPTIMIZER
# #############################################################################
def optimize_offsets(queue_lengths, cycle_length, green_phases, adaptation_factor, threshold):
    """
    3. Optimize offsets to enable green waves
    """
    global offsets, downstream_junction_map, most_congested_lane, estimated_travel_time
    estimated_travel_time = {}
    # Define three traffic districts
    districts = {
        "front": ["intersection1","intersection2"],
        "middle": ["intersection3", "intersection4"],
        "back": ["intersection5"]
    }
    # Compute congestion per district (normalized by edge count)
    district_congestion = {d: 0 for d in districts}
    district_length_count = {d: 0 for d in districts}
    for district, junctions in districts.items():
        for j in junctions:
            district_congestion[district] += sum(queue_lengths[j].values())
            district_length_count[district] += sum([traci.lane.getLength(lane) for lane in queue_lengths[j].keys()])
        if district == "back":
            #district_congestion[district] += sum(queue_lengths["intersection5"].get(lane, 0) for lane in ["E13_1","E8_1"])
            district_congestion[district] /= (district_length_count[district])
        elif district == "front":
            #district_congestion[district] += sum(queue_lengths["intersection1"].get(lane, 0) for lane in ["-183419042#1_1"])
            district_congestion[district] /= (district_length_count[district])
        else:
            #district_congestion[district] += sum(queue_lengths["J5"].get(lane, 0) for lane in ["-183419042#1_1"])
            district_congestion[district] /= (district_length_count[district])
        district_congestion[district] *= 10
    critical_district = max(district_congestion, key=district_congestion.get)
    sorted_congestion = sorted(district_congestion.values(), reverse=True)
    congestion_gap = abs(sorted_congestion[0] - sorted_congestion[1])
    # Estimate travel time between junctions
    for junction in connection.keys():
        speed = 8.33  # m/s
        length = sum([traci.lane.getLength(lane) for lane in connection[junction]])
        if junction == "intersection1":
            length += 2*traci.lane.getLength("183049934_1")
        if junction == "intersection3":
            length += 3*traci.lane.getLength("E1_1")
        if junction == "intersection4":
            length += 9*traci.lane.getLength("22889927#3_1")
        estimated_travel_time[junction] = length / speed
    # Determine downstream junction ordering
    if critical_district == "front":
        ordered_junctions = ["intersection1", "intersection2", "intersection3", "intersection4", "intersection5"]
    elif critical_district == "back":
        ordered_junctions = ["intersection5", "intersection4", "intersection3", "intersection2", "intersection1"]
    else:
        ordered_junctions = ["intersection3", "intersection2", "intersection4", "intersection1", "intersection5"]
    if congestion_gap > threshold:
        offsets = {}
        previous_junction = None
        for j in ordered_junctions:
            if previous_junction is None:
                offsets[j] = 0
            else:
                if critical_district == "front":
                    offsets[j] = min(offsets[previous_junction] + estimated_travel_time.get(previous_junction, 0) * adaptation_factor,
                                 cycle_length)
                elif critical_district == "back":
                    offsets[j] = min(offsets[previous_junction] + estimated_travel_time.get(j, 0) * adaptation_factor,
                      cycle_length)     
                else:
                    if j == "intersection2":
                        offsets[j] = min(estimated_travel_time.get(j, 0) * adaptation_factor,
                                         cycle_length)       
                    elif j == "intersection4":
                        offsets[j] = min(estimated_travel_time.get("intersection3", 0) * adaptation_factor,
                                         cycle_length)                   
                    elif j == "intersection1":
                        offsets[j] = min(offsets["intersection2"] + estimated_travel_time.get(j, 0) * adaptation_factor,
                                         cycle_length)
                    else:
                        offsets[j] = min(offsets["intersection4"] + estimated_travel_time.get("intersection4", 0) * adaptation_factor,
                                         cycle_length)
            previous_junction = j
    elif congestion_gap < threshold - 0.1: # Hysteresis for stability
        offsets = {j: 0 for j in ordered_junctions}
    return offsets

# #############################################################################
# ## APPLY OFFSETS, GREENPHASES
# #############################################################################
def setup_scosca_control(queue_lengths, degree_of_sat, step,
                             adaptation_cycle, adaptation_green, green_thresh,
                             adaptation_offset, offset_thresh,
                             greentimes,cycle_length):
    """
    4. Main function: Apply SCOSCA traffic signal logic
    """
    global offsets, update_counter
    if update_counter % 5 == 0 and update_counter != 0:
        cycle_length = optimize_cycle_length(degree_of_sat, cycle_length, adaptation_cycle)
    if update_counter != 0:
        greentimes = optimize_green_phases(queue_lengths, degree_of_sat, cycle_length,greentimes, adaptation_green, green_thresh)
    if update_counter % 5 == 0 and update_counter != 0:
        offsets = optimize_offsets(queue_lengths, cycle_length, greentimes, adaptation_offset, offset_thresh)
    update_counter += 1
    # Apply logic to each junction
    for i, junction in enumerate(greentimes.keys()):
        greens = greentimes[junction]
        phases = []
        for idx, g_duration in enumerate(greens):
            phases.append(traci.trafficlight.Phase(g_duration, green_states[junction][idx]))
            phases.append(traci.trafficlight.Phase(3, yellow_states[junction][idx]))  # fixed yellow duration
        logic = traci.trafficlight.Logic(
            programID=f"program_fixed_{i}",
            type=tc.TRAFFICLIGHT_TYPE_STATIC,
            currentPhaseIndex=0,
            phases=phases
        )
        traci.trafficlight.setProgramLogic(junction, logic)
        # Apply offsets for green wave alignment
        shift = int(offsets.get(junction, 0))
        if shift == 0:
            traci.trafficlight.setPhase(junction, 0)
        elif shift < 3:
            traci.trafficlight.setPhase(junction, len(greens)*2-1)
            traci.trafficlight.setPhaseDuration(junction, shift)
        elif shift < greens[-1] + 3:
            traci.trafficlight.setPhase(junction, len(greens)*2-2)
            traci.trafficlight.setPhaseDuration(junction, shift - 3)
        elif shift < greens[-1] + 6:
            traci.trafficlight.setPhase(junction, len(greens)*2-3)
            traci.trafficlight.setPhaseDuration(junction, shift - greens[-1] - 3)
        elif shift < greens[-1] + 6 + greens[-2]:
            traci.trafficlight.setPhase(junction, len(greens)*2-4)
            traci.trafficlight.setPhaseDuration(junction, shift - greens[-1] - 6)
        else:
            if len(greens)*2 > 4:
                if shift < greens[-1]+9+greens[-2]:
                    traci.trafficlight.setPhase(junction, len(greens)*2-5)
                    traci.trafficlight.setPhaseDuration(junction, shift - greens[-1]-greens[-2] - 6)
                elif shift < greens[-1] + 9 +greens[-2]+greens[-3]:
                    traci.trafficlight.setPhase(junction, len(greens)*2-6)
                    traci.trafficlight.setPhaseDuration(junction, shift - greens[-1]-greens[-2] - 9) 
                else:
                    print("ERROR1",flush=True)
                    print(f"offsets: {offsets}",flush=True)
                    print(f"junction: {junction}",flush=True)
                    print(f"greens: {greens}",flush=True)
            else:
                print("ERROR2",flush=True)
                print(f"offsets: {offsets}",flush=True)
                print(f"greentimes: {greens}",flush=True)
    return cycle_length, greentimes
