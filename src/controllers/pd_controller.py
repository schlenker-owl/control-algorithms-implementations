import numpy as np

class PDController:
    """
    Proportional-Derivative (PD) controller.
    """
    def __init__(self, Kp=1.0, Kd=0.0, dt=0.01):
        """
        :param Kp: Proportional gain
        :param Kd: Derivative gain
        :param dt: Sampling time
        """
        self.Kp = Kp
        self.Kd = Kd
        self.dt = dt
        self.prev_error = 0.0

    def reset(self):
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        
        # Proportional
        P_out = self.Kp * error
        
        # Derivative (finite difference)
        d_error = (error - self.prev_error) / self.dt
        D_out = self.Kd * d_error
        
        # Save state
        self.prev_error = error
        
        return P_out + D_out
