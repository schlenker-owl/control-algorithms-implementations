class InnerPID:
    """
    Simple PID for the inner loop in a cascade arrangement.
    """
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01, windup_limit=1e6):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.windup_limit = windup_limit

        self.int_err = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.int_err = 0.0
        self.prev_err = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        P_out = self.Kp * error
        
        self.int_err += error * self.dt
        self.int_err = max(min(self.int_err, self.windup_limit), -self.windup_limit)
        I_out = self.Ki * self.int_err

        d_err = (error - self.prev_err) / self.dt
        D_out = self.Kd * d_err

        self.prev_err = error
        return P_out + I_out + D_out

class OuterPID:
    """
    Simple PID for the outer loop in a cascade arrangement.
    """
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01, windup_limit=1e6):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.windup_limit = windup_limit

        self.int_err = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.int_err = 0.0
        self.prev_err = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        P_out = self.Kp * error
        
        self.int_err += error * self.dt
        self.int_err = max(min(self.int_err, self.windup_limit), -self.windup_limit)
        I_out = self.Ki * self.int_err

        d_err = (error - self.prev_err) / self.dt
        D_out = self.Kd * d_err

        self.prev_err = error
        return P_out + I_out + D_out
