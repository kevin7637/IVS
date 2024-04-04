class PD_Controller(object):
    def __init__(self,measure, step_time, P_Gain=0.6, D_Gain=1.2):
        self.kp = P_Gain
        self.kd = D_Gain
        reference = 0
        self.pre = reference - measure
        self.step_time = step_time
        self.u = 0.0
            
    def ControllerInput(self,measure):
        reference = 0
        error = reference - measure
        error_d = (error - self.error_pre)/self.step_time
        self.u = self.kp*error + self.kd*error_d
        self.pre = error
        

def yaw_controll(error):
    #300이 0도, 100 우회전, 500이 좌회전
    #오차는 lane_width만큼 생김 좌 0.2m, 우 -0.2
    #yaw값은 servo_tick = (-error + 0.3) * (500 - 100) / (0.3 - (-0.3)) + 100
    RIGHT_MAX = 500
    LEFT_MAX = 100
    MAX_ERROR = 0.3
    servo_tick = (-error + MAX_ERROR) * (RIGHT_MAX - LEFT_MAX) / (MAX_ERROR - (-MAX_ERROR)) + LEFT_MAX
    
    if servo_tick > 500:
        servo_tick = 500
    if servo_tick  <100:
        servo_tick = 100
    
    return servo_tick

    