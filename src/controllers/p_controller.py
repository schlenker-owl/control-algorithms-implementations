class PController:
    """
    Simple Proportional (P) controller.
    """
    def __init__(self, Kp=1.0):
        """
        :param Kp: Proportional gain
        """
        self.Kp = Kp

    def compute(self, setpoint, measurement):
        """
        Computes the control output based on a setpoint and measurement.
        :param setpoint: Desired value
        :param measurement: Current measured value
        :return: Control output (float)
        """
        error = setpoint - measurement
        control_output = self.Kp * error
        return control_output
