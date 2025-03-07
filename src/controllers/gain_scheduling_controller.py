import numpy as np

class GainSchedulingPID:
    """
    A PID controller whose gains (Kp, Ki, Kd) depend on a scheduling variable,
    e.g. speed, temperature, etc.

    We'll do a simple "table lookup" approach with linear interpolation.
    """

    def __init__(self, dt=0.01, windup_limit=1e6):
        self.dt = dt
        self.windup_limit = windup_limit

        self.integral_term = 0.0
        self.prev_error = 0.0

        # Gains schedule: list of tuples (sched_value, Kp, Ki, Kd)
        self.gain_table = []

    def add_gain_point(self, sched_value, Kp, Ki, Kd):
        """
        Add a point (sched_value -> (Kp, Ki, Kd)) to the schedule.
        Make sure to add them in ascending order of sched_value.
        """
        self.gain_table.append((sched_value, Kp, Ki, Kd))
        self.gain_table.sort(key=lambda x: x[0])  # keep sorted

    def _interpolate_gains(self, current_sched):
        """
        Interpolate between the two nearest points in self.gain_table
        based on current_sched.
        """
        # If out of bounds, clamp to min or max
        if current_sched <= self.gain_table[0][0]:
            return self.gain_table[0][1:]
        if current_sched >= self.gain_table[-1][0]:
            return self.gain_table[-1][1:]

        # Otherwise, find bracketing points
        for i in range(len(self.gain_table)-1):
            s0, kp0, ki0, kd0 = self.gain_table[i]
            s1, kp1, ki1, kd1 = self.gain_table[i+1]

            if s0 <= current_sched <= s1:
                # linear interpolation
                ratio = (current_sched - s0)/(s1 - s0)
                Kp = kp0 + ratio*(kp1 - kp0)
                Ki = ki0 + ratio*(ki1 - ki0)
                Kd = kd0 + ratio*(kd1 - kd0)
                return (Kp, Ki, Kd)

        # fallback (should not happen if logic is correct)
        return self.gain_table[-1][1:]

    def reset(self):
        self.integral_term = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement, sched_value):
        """
        :param setpoint: desired output
        :param measurement: actual output
        :param sched_value: current scheduling parameter
        """
        error = setpoint - measurement

        # get current gains by interpolation
        Kp, Ki, Kd = self._interpolate_gains(sched_value)

        # P
        P_out = Kp * error

        # I
        self.integral_term += error * self.dt
        self.integral_term = np.clip(self.integral_term, -self.windup_limit, self.windup_limit)
        I_out = Ki * self.integral_term

        # D
        derivative = (error - self.prev_error) / self.dt
        D_out = Kd * derivative

        self.prev_error = error
        return P_out + I_out + D_out
