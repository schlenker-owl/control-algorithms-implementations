import numpy as np

class PIDController:
    """
    Simple PID controller class.
    """
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01, windup_limit=1e6):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.windup_limit = windup_limit

        self.integral_term = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral_term = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        
        # Proportional
        P_out = self.Kp * error
        
        # Integral
        self.integral_term += error * self.dt
        # Anti-windup
        self.integral_term = np.clip(self.integral_term, -self.windup_limit, self.windup_limit)
        I_out = self.Ki * self.integral_term
        
        # Derivative
        derivative = (error - self.prev_error) / self.dt
        D_out = self.Kd * derivative
        
        # Combine
        output = P_out + I_out + D_out
        
        # Save state
        self.prev_error = error
        
        return output
