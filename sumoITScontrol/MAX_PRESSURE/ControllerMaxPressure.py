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
    This script contains the implementation of the MaxPressure traffic
    light controller.
"""




# #############################################################################
# ###### IMPORTS ##############################################################
# #############################################################################
import traci
import random
import pandas as pd




# #############################################################################
# ###### MAX PRESSURE PARAMETER ###############################################
# #############################################################################
T_A = 5 
T_L = 3 #Yellow Time
G_T_MIN = 5 #Min Greentime (used for Max. Pressure)
G_T_MAX = 50 #Max Greentime (used for Max. Pressure)
WEIGHTS_MAX_PRESSURE = {"car": 1.0, "moc": 1.0, "lwt": 1.0, "hwt": 1.0, "bus": 1.0}

global df_current_status, df_hidden_vehicles
df_current_status = None
df_hidden_vehicles = None




# #############################################################################
# ## MAX PRESSURE CONTROLLER
# #############################################################################
class MaxPressure_SignalController:
    def __init__(self, intersection_name, phases, links, multiplier=None):
        self.intersection_name = intersection_name
        self.phases = phases
        self.links = links
        self.current_gt_start = 0
        self.current_phase = self.phases[0]
        self.next_phase = -1
        self.current_state = "start"
        self.timer = -1
        self.pressures = []
        self.multiplier = multiplier
        
    def do_signal_logic(self):
        self.timer += 1
        self.determine_pressures()
        if self.current_state == "start":
            if self.timer==G_T_MIN:
                self.current_state="check_pressures"
                self.timer = -1
            else:
                pass
        elif self.current_state=="check_pressures":
            current_pressure = self.pressures[int(self.current_phase/2)]
            other_pressures = max(self.pressures)
            if current_pressure < other_pressures:
                self.current_state="next_phase"
                self.timer = -1
            else:
                self.current_state="wait"
                self.timer = -1
        elif self.current_state=="wait":
            if self.timer==T_A:
                current_gt = traci.simulation.getTime()-self.current_gt_start
                if current_gt > G_T_MAX:
                    self.current_state = "next_phase"
                    self.timer = -1
                else:
                    self.current_state="check_pressures"
                    self.timer = -1
            else:
                pass
        elif self.current_state=="next_phase":
            valid_indices = [i for i in range(len(self.pressures)) if i != int(self.current_phase/2)]
            max_pressure = max(self.pressures[i] for i in valid_indices)
            max_indices = [i for i in valid_indices if self.pressures[i] == max_pressure]
            self.next_phase = int(random.choice(max_indices)*2)
            self.current_phase += 1
            self.timer = -1
            self.current_state="transition"
        elif self.current_state=="transition":
            if self.timer==T_L:
                self.current_phase = self.next_phase 
                self.next_phase = -1
                self.timer = -1
                self.current_state = "start"
                self.current_gt_start = traci.simulation.getTime()
            else:
                pass
        else:
            print("WARNING UNKNOWN STATE", self.current_state)
            print("")
        self.set_signal_on_traffic_lights()
            
    def determine_pressures(self):
        if df_current_status is None:
            self.pressures = [0 for p in self.links]
            return
        self.pressures = []
        for link in self.links:
            lanes = self.links[link]
            df_vehicles = []
            # based on lane
            if type(df_vehicles)==list:
                df_vehicles = df_current_status[df_current_status["lane"].isin(lanes)]
            else:
                df_vehicles = pd.concat((df_vehicles, df_current_status[df_current_status["lane"].isin(lanes)]))
            # based on hidden on intersection
            edges = [l.split("_")[0] for l in lanes]
            hits = df_hidden_vehicles[df_hidden_vehicles["edge"].isin(edges)]
            if len(hits)>0:
                if type(df_vehicles)==list:
                    df_vehicles = df_hidden_vehicles[df_hidden_vehicles["edge"].isin(edges)]
                else:
                    df_vehicles = pd.concat((df_vehicles, df_hidden_vehicles[df_hidden_vehicles["edge"].isin(edges)][["veh_id", "lane", "class", "weight"]]))
            pressure = 0 
            if len(df_vehicles)>0:
                pressure = sum(df_vehicles["weight"])
            # multiplier
            if self.multiplier is not None:
                if link in self.multiplier:
                    pressure *= self.multiplier[link]
            self.pressures.append(pressure)
    
    def set_signal_on_traffic_lights(self):
        traci.trafficlight.setPhase(self.intersection_name, self.current_phase)

controller1 = MaxPressure_SignalController(
    intersection_name = "intersection1",
    phases = [0, 2, 4],
    links = {0:["921020465#1_3", "921020465#1_2", "921020464#0_1", "921020464#1_1", "38361907_3", "38361907_2", "-1164287131#1_3", "-1164287131#1_2"], 
             2:["-1169441386_2", "-1169441386_1", "-331752492#1_2", "-331752492#1_1", "-331752492#0_1", "-331752492#0_2"], 
             4:["-183419042#1_1", "26249185#30_1", "26249185#30_2", "26249185#1_1", "26249185#1_2"]},
    )

controller2 = MaxPressure_SignalController(
    intersection_name = "intersection2",
    phases = [0, 2, 4],
    links = {0:["183049933#0_1", "-38361908#1_1"], 
             2:["-38361908#1_1", "-38361908#1_2"], 
             4:["-25973410#1_1", "758088375#0_1", "758088375#0_2"]}
    )

controller3 = MaxPressure_SignalController(
    intersection_name = "intersection3",
    phases = [0, 2, 4],
    links = {0:["E3_1", "-758088377#1_1", "-758088377#1_2", "-E1_1", "-E1_2"], 
             2:["E3_1", "E3_2"], 
             4:["-758088377#1_1", "-E1_1", "-E4_1", "-E4_2"]}
    )

controller4 = MaxPressure_SignalController(
    intersection_name = "intersection4",
    phases = [0, 2],
    links = {0:["22889927#0_1", "758088377#2_1", "-22889927#2_1"], 
             2:["-25576697#0_0"]}
    )

controller5 = MaxPressure_SignalController(
    intersection_name = "intersection5",
    phases = [0, 2, 4],
    links = {0:["E6_1", "E6_2", "E5_1", "130569446_1", "E15_1", "E15_2"], 
             2:["E15_2", "E6_3", "E5_2", "130569446_2"],
             4:["E10_1", "E9_1",  "1162834479#1_1", "-208691154#0_1", "-208691154#1_1"]},
    )
signal_controllers = [controller1, controller2, controller3, controller4, controller5]