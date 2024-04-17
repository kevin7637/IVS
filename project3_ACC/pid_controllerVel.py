class PIDControllerVel:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.prev_time = None

    def update(self, ref, measure, current_time):
        # 현재 오차 계산
        error = ref - measure
        
        # 시간 간격 (dt) 계산
        if self.prev_time is None:
            dt = 0
        else:
            dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # P 성분: 현재 오차
        P_out = self.Kp * error
        
        # I 성분: 오차의 누적값
        self.integral += error * dt
        I_out = self.Ki * self.integral
        
        # D 성분: 오차의 변화율
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        D_out = self.Kd * derivative
        
        # 이전 오차 업데이트
        self.prev_error = error

        # 제어 값 계산
        control_value = P_out + I_out + D_out
        return control_value
