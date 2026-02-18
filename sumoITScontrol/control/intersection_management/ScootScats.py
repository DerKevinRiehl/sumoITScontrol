# sumoITScontrol/control/intersection_management/ScootScats.py

from sumoITScontrol import IntersectionGroup
from sumoITScontrol import SimulationTools


class ScootScats:
    def __init__(
        self,
        params,
        intersection_group: IntersectionGroup,
        initial_greentimes,
        initial_cycle_length=120,
    ):
        self.params = params
        self.intersection_group = intersection_group
        # init measurement data
        self.measurement_data = {}
        self.measurement_data["measurement_counter"] = -1
        self.measurement_data["control_counter"] = 0
        self.measurement_data["update_counter"] = 0
        self.measurement_data["last_cycle_update"] = 0
        self.measurement_data["cycle_length"] = initial_cycle_length
        self.measurement_data["previous_cycle_length"] = {
            i.tl_id: initial_cycle_length for i in self.intersection_group.intersections
        }
        self.measurement_data["greentimes"] = {
            k: list(v) for k, v in initial_greentimes.items()
        }
        self.measurement_data["offsets"] = {
            i.tl_id: 0 for i in self.intersection_group.intersections
        }
        self.measurement_data["estimated_travel_time"] = {}

    def optimize_cycle_length(self, degree_of_sat):
        """
        This function optimizes the total cycle length of the whole intersection group.
        """
        max_ds_in_network = max(max(lane.values()) for lane in degree_of_sat.values())
        if (
            max_ds_in_network >= self.params["ds_upper_val"]
        ):  # Increase cycle length if highly saturated (SCOOT principle)
            new_length = (
                self.measurement_data["cycle_length"]
                + (max_ds_in_network - self.params["ds_upper_val"])
                * self.params["adaptation_cycle"]
            )
            self.measurement_data["cycle_length"] = min(
                int(new_length), self.params["max_cycle_length"]
            )
        elif (
            0 < max_ds_in_network < self.params["ds_lower_val"]
        ):  # Decrease if network is underused
            new_length = (
                self.measurement_data["cycle_length"]
                - (self.params["ds_lower_val"] - max_ds_in_network)
                * self.params["adaptation_cycle"]
            )
            self.measurement_data["cycle_length"] = max(
                int(new_length), self.params["min_cycle_length"]
            )
        else:  # If DS is in a good range, keep the current cycle
            self.measurement_data["cycle_length"] = self.measurement_data[
                "cycle_length"
            ]

    def optimize_green_phases(self, queue_lengths, degree_of_sat):
        """
        This function optimizes the green phase durations for each intersection.
        """
        for intersection, greens in list(self.measurement_data["greentimes"].items()):
            effective_cycle = self.measurement_data["cycle_length"] - 3 * len(
                greens
            )  # Subtract yellow phases
            # Get the lane with the highest degree of saturation
            max_lane = max(
                degree_of_sat[intersection], key=degree_of_sat[intersection].get
            )
            max_queue = queue_lengths[intersection].get(max_lane, 0)
            max_ds = degree_of_sat[intersection].get(max_lane, 0)
            if max_queue > self.params["green_thresh"]:
                # Identify all phases where max_lane appears
                phases_with_max_lane = [
                    p
                    for p, lane_list in self.intersection_group.get_intersection(
                        intersection
                    ).links.items()
                    if max_lane in lane_list
                ]
                # Determine the phase of max_lane
                if len(phases_with_max_lane) == 1:
                    max_phase = phases_with_max_lane[0] // 2
                else:
                    max_phase = 0  # fallback if lane is used in multiple phases
                # Collect all lanes in those phases (to exclude them)
                excluded_lanes = set()
                for p in phases_with_max_lane:
                    excluded_lanes.update(
                        self.intersection_group.get_intersection(intersection).links[p]
                    )
                # Get DS per other phase (not containing max_lane)
                phase_ds_list = []
                for phase in range(len(greens)):
                    if phase == max_phase:
                        continue
                    lanes = self.intersection_group.get_intersection(
                        intersection
                    ).links[phase * 2]
                    phase_lanes = [l for l in lanes if l not in excluded_lanes]
                    if phase_lanes:
                        ds_val = max(
                            [degree_of_sat[intersection].get(l, 0) for l in phase_lanes]
                        )
                    else:
                        ds_val = 0
                    phase_ds_list.append((phase, ds_val))
                # Sort DS
                phase_ds_sorted = sorted(
                    phase_ds_list, key=lambda x: x[1], reverse=True
                )
                second_phase = phase_ds_sorted[0][0]
                second_ds = phase_ds_sorted[0][1]
                third_phase = (
                    phase_ds_sorted[1][0] if len(phase_ds_sorted) > 1 else None
                )
                third_ds = (
                    phase_ds_sorted[1][1] if len(phase_ds_sorted) > 1 else second_ds
                )
                # Compute DS differences
                ds_diff_max = max(0, max_ds - third_ds)
                ds_diff_second = max(0, second_ds - third_ds)
                self.measurement_data["greentimes"][intersection][max_phase] = int(
                    min(
                        3 * effective_cycle / 4,
                        self.measurement_data["greentimes"][intersection][max_phase]
                        + ds_diff_max * self.params["adaptation_green"],
                    )
                )
                # Assign the remaining green time
                remaining_time = (
                    effective_cycle
                    - self.measurement_data["greentimes"][intersection][max_phase]
                )
                if len(greens) == 2:
                    # Only one other phase — assign remaining time to it
                    self.measurement_data["greentimes"][intersection][
                        second_phase
                    ] = remaining_time
                elif len(greens) == 3:
                    # Determine saturation levels for the two remaining phases
                    self.measurement_data["greentimes"][intersection][second_phase] = (
                        int(
                            min(
                                2 * remaining_time / 3,
                                self.measurement_data["greentimes"][intersection][
                                    second_phase
                                ]
                                + ds_diff_second * self.params["adaptation_green"],
                            )
                        )
                    )
                    self.measurement_data["greentimes"][intersection][third_phase] = (
                        remaining_time
                        - self.measurement_data["greentimes"][intersection][
                            second_phase
                        ]
                    )
            else:
                # Only scale with cycle length
                scaled = [
                    int(
                        g
                        * effective_cycle
                        / self.measurement_data["previous_cycle_length"][intersection]
                    )
                    for g in greens
                ]
                # Normalize to match effective_cycle exactly
                diff = effective_cycle - sum(scaled)
                scaled[0] += diff  # Adjust first to balance
                self.measurement_data["greentimes"][intersection] = scaled
            self.measurement_data["previous_cycle_length"][intersection] = (
                self.measurement_data["cycle_length"] - 3 * len(greens)
            )

    def optimize_offsets(self, queue_lengths):
        """
        This function optimizes offsets across intersection programmes to enable green waves.
        """
        self.measurement_data["estimated_travel_time"] = {}
        # Compute congestion per district (normalized by edge count)
        district_congestion = {d: 0 for d in self.intersection_group.districts}
        district_length_count = {d: 0 for d in self.intersection_group.districts}
        for district, intersections in self.intersection_group.districts.items():
            for intersection in intersections:
                district_congestion[district] += sum(
                    queue_lengths[intersection].values()
                )
                district_length_count[district] += sum(
                    [
                        SimulationTools.get_lane_length_preloaded(lane)
                        for lane in queue_lengths[intersection].keys()
                    ]
                )
            district_congestion[district] /= district_length_count[district]
            district_congestion[district] *= 10
        critical_district = max(district_congestion, key=district_congestion.get)
        sorted_congestion = sorted(district_congestion.values(), reverse=True)
        congestion_gap = abs(sorted_congestion[0] - sorted_congestion[1])
        # Estimate travel time between intersections and adjust
        for (
            intersection
        ) in self.intersection_group.connection_between_intersections.keys():
            length = sum(
                [
                    SimulationTools.get_lane_length_preloaded(lane)
                    for lane in self.intersection_group.connection_between_intersections[
                        intersection
                    ]
                ]
            )
            if intersection in self.params["travel_time_adjustments"]:
                length += self.params["travel_time_adjustments"][intersection][
                    1
                ] * SimulationTools.get_lane_length_preloaded(
                    self.params["travel_time_adjustments"][intersection][0]
                )
            self.measurement_data["estimated_travel_time"][intersection] = (
                length / self.intersection_group.speed_limit
            )
        # Determine downstream intersection ordering
        ordered_intersections = self.intersection_group.critical_district_order[
            critical_district
        ]
        if congestion_gap > self.params["offset_thresh"]:
            self.measurement_data["offsets"] = {}
            previous_intersection = None
            for intersection in ordered_intersections:
                if previous_intersection is None:
                    self.measurement_data["offsets"][intersection] = 0
                else:
                    if critical_district == "front":
                        self.measurement_data["offsets"][intersection] = min(
                            self.measurement_data["offsets"][previous_intersection]
                            + self.measurement_data["estimated_travel_time"].get(
                                previous_intersection, 0
                            )
                            * self.params["adaptation_offset"],
                            self.measurement_data["cycle_length"],
                        )
                    elif critical_district == "back":
                        self.measurement_data["offsets"][intersection] = min(
                            self.measurement_data["offsets"][previous_intersection]
                            + self.measurement_data["estimated_travel_time"].get(
                                intersection, 0
                            )
                            * self.params["adaptation_offset"],
                            self.measurement_data["cycle_length"],
                        )
                    else:
                        rule = self.params["intersection_offset_rules"].get(
                            intersection, "default"
                        )
                        base_offset = 0
                        if rule["base_offset_from"] is not None:
                            base_offset = self.measurement_data["offsets"][
                                rule["base_offset_from"]
                            ]

                        travel_time = self.measurement_data[
                            "estimated_travel_time"
                        ].get(rule["travel_time_from"], 0)

                        self.measurement_data["offsets"][intersection] = min(
                            base_offset
                            + travel_time * self.params["adaptation_offset"],
                            self.measurement_data["cycle_length"],
                        )
                previous_intersection = intersection
        elif (
            congestion_gap < self.params["offset_thresh"] - 0.1
        ):  # Hysteresis for stability
            self.measurement_data["offsets"] = {
                intersection: 0 for intersection in ordered_intersections
            }

    def execute_control(self, current_time):
        self.measurement_data["measurement_counter"] += 1
        if (
            self.measurement_data["measurement_counter"]
            == self.params["measurement_period"]
        ):
            self.measurement_data["measurement_counter"] = 0

            self.measurement_data["control_counter"] += 1
            if (
                self.measurement_data["control_counter"]
                == self.measurement_data["last_cycle_update"]
                + self.measurement_data["cycle_length"]
            ):
                self.measurement_data["last_cycle_update"] = self.measurement_data[
                    "control_counter"
                ]

                # measure state from SUMO
                queue_lengths, degree_of_sat = (
                    self.intersection_group.measure_queues_and_ds()
                )

                # call optimizers according to schedule
                if (
                    self.measurement_data["update_counter"] % 5 == 0
                    and self.measurement_data["update_counter"] != 0
                ):
                    self.optimize_cycle_length(degree_of_sat)
                if self.measurement_data["update_counter"] != 0:
                    self.optimize_green_phases(queue_lengths, degree_of_sat)
                if (
                    self.measurement_data["update_counter"] % 5 == 0
                    and self.measurement_data["update_counter"] != 0
                ):
                    self.optimize_offsets(queue_lengths)

                self.measurement_data["update_counter"] += 1
                self.intersection_group.apply_tl_programme(
                    self.measurement_data["greentimes"],
                    self.measurement_data["offsets"],
                )
