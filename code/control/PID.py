class PD_Controller(object):
    def __init__(self,measure, step_time, P_Gain=0.01, D_Gain=0.1):
        self.kp = P_Gain
        self.kd = D_Gain
        reference = 0
        self.error_pre = reference - measure
        self.step_time = step_time
        self.u = 0.0
        self.servo_tick = 0
            
    def ControllerInput(self,measure):
        reference = 0
        error = reference - measure
        error_d = (error - self.error_pre)/self.step_time
        self.u = self.kp*error + self.kd*error_d
        self.error_pre = error
        

    