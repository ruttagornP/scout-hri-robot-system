class PIDController:

    def __init__(self, kp, ki, kd, min_output=None, max_output=None):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output

        self.prev_error = 0.0
        self.integral = 0.0
        
    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, error, dt=1.0):

        self.integral += error * dt

        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        
        if self.min_output is not None:
            output = max(self.min_output, output)

        if self.max_output is not None:
            output = min(self.max_output, output)

        return output

    def reset(self):

        self.prev_error = 0.0
        self.integral = 0.0