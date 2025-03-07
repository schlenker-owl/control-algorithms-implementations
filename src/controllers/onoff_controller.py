class OnOffController:
    """
    Simple on-off (bang-bang) controller with hysteresis.
    """

    def __init__(self, on_threshold, off_threshold):
        """
        :param on_threshold: If error > on_threshold, controller output = 1
        :param off_threshold: If error < off_threshold, controller output = 0
        """
        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.output_state = 0.0  # track current on/off state

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        
        # Switch ON if error is above on_threshold
        if error > self.on_threshold:
            self.output_state = 1.0
        # Switch OFF if error is below off_threshold
        elif error < self.off_threshold:
            self.output_state = 0.0

        return self.output_state
