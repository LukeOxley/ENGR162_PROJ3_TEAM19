class Hazard:
    def __init__(self, hazard_type, hazard_param, hazard_value, point):
        self.hazard_type = hazard_type
        self.hazard_param = hazard_param
        self.hazard_value = hazard_value
        self.point = point
    def __str__(self):
        return str(self.hazard_type)+"with"+str(self.hazard_param)+" of "+str(self.hazard_value)+" at "+str(self.point)  

class Hazard_Type_t:
    BIOHAZARD = "BIOHAZARD"

class Hazard_Param_t:
    MASS = "MASS"

