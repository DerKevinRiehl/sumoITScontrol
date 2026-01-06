
class SimulationTools:
    sensor_list_initialized = False
    sensor_list_e1_inductionloops = []
    sensor_list_e2_laneareas = [] 
    
    @staticmethod
    def init_sensor_lists(traci):
        SimulationTools.sensor_list_e1_inductionloops = traci.inductionloop.getIDList()
        SimulationTools.sensor_list_e2_laneareas = traci.lanearea.getIDList()
        SimulationTools.sensor_list_initialized = True
        
    @staticmethod
    def get_sensor_type(sensor_id: str) -> str | None:
        if sensor_id in SimulationTools.sensor_list_e1_inductionloops:
            return "inductionloop"
        if sensor_id in SimulationTools.sensor_list_e2_laneareas:
            return "lanearea"
        return None