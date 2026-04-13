import numpy as np

class FuelControllerPID:
    def __init__(self, Kp, Ki, Kd, target_lambda=14.7):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target_lambda
        self.prev_error = 0
        self.integral = 0

    def update(self, measured_lambda, dt):
        error = self.target - measured_lambda
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        # Calculate fuel adjustment
        adjustment = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        
        # Return total fuel command (Base + Adjustment)
        # Assuming a base fuel flow proportional to air flow is handled in the bench
        return adjustment

