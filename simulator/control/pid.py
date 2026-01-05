class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, ref, measured_value, step_time):
        error = ref - measured_value
        self.integral += error*step_time
        derivative = (error - self.prev_error)/step_time
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative