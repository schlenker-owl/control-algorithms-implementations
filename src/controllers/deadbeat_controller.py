import numpy as np

class DeadbeatController:
    """
    A simple discrete-time deadbeat controller for a first-order system
    of the form x[k+1] = a*x[k] + b*u[k].
    The goal is to place the closed-loop pole at 0 (or near zero),
    achieving deadbeat (one-step) response.
    """

    def __init__(self, a, b):
        """
        :param a: System coefficient
        :param b: Input coefficient
        """
        self.a = a
        self.b = b
        self.prev_output = 0.0

        # For a 1st-order system x[k+1] = a*x[k] + b*u[k],
        # we want x[k+1] = 0 in 1 step if we aim for deadbeat to zero.
        # If the desired next state is r[k+1], we solve for u = (r[k+1] - a*x[k])/b.
        # For tracking a setpoint, we might do a simpler approach:
        # Let setpoint = r. We want x[k+1] = r => u = (r - a*x[k])/b.

    def compute(self, setpoint, measurement):
        """
        For a 1st-order model x[k+1] = a*x[k] + b*u[k],
        we want x[k+1] = setpoint => u = (setpoint - a*measurement) / b.
        """
        u = (setpoint - self.a*measurement) / self.b
        return u
