class FeedForwardController:
    """
    Simple feed-forward + PID combination.
    The feed-forward term aims to invert the plant model (or partial),
    while the PID handles error correction.
    """

    def __init__(self, pid_controller, feedforward_gain=1.0):
        """
        :param pid_controller: an existing PID-like controller instance
        :param feedforward_gain: a scalar or function that approximates G^-1
        """
        self.pid = pid_controller
        self.ff_gain = feedforward_gain

    def reset(self):
        self.pid.reset()

    def compute(self, setpoint, measurement):
        # feed-forward part (assuming setpoint is the desired output, so input ~ G^-1 * setpoint)
        ff = self.ff_gain * setpoint
        
        # feedback part
        fb = self.pid.compute(setpoint, measurement)
        
        return ff + fb
