import numpy as np

class PIController:
    """
    Proportional-Integral (PI) controller.
    """
    def __init__(self, Kp=1.0, Ki=0.0, dt=0.01, windup_limit=1e6):
        """
        :param Kp: Proportional gain
        :param Ki: Integral gain
        :param dt: Sampling time
        :param windup_limit: Anti-windup clamp
        """
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        self.windup_limit = windup_limit

        self.integral_term = 0.0

    def reset(self):
        self.integral_term = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        
        # Proportional
        P_out = self.Kp * error
        
        # Integral
        self.integral_term += error * self.dt
        self.integral_term = np.clip(self.integral_term, -self.windup_limit, self.windup_limit)
        I_out = self.Ki * self.integral_term
        
        return P_out + I_out
