import control as ctrl

class SmithPredictor:
    """
    A simple Smith Predictor structure to handle plants with time delay.
    
    The idea:
    1. We have a plant: Gp(s) * e^{-Ls}, with dead time L.
    2. We define a nominal model Gm(s) (no delay) + known delay e^{-Ls}.
    3. A "normal" controller C(s) is designed for the no-delay portion Gm(s).
    4. The Smith Predictor compensates the effect of the real plant's delay
       by using the model internally, so the feedback sees a 'no delay' system.
    """

    def __init__(self, controller_no_delay, plant_nominal, delay):
        """
        :param controller_no_delay: A python-control TransferFunction or StateSpace 
                                    that represents the controller for the no-delay model.
        :param plant_nominal: The nominal plant model Gm(s) (no delay).
        :param delay: Time delay (seconds).
        """
        self.C = controller_no_delay
        self.Gm = plant_nominal
        self.delay = delay

    def closed_loop_tf(self):
        """
        Build the closed-loop transfer function with the Smith Predictor structure:
        
        Output(s)/Reference(s) = [C(s)*Gm(s)] / [1 + C(s)*[Gp(s) - Gm(s)] + C(s)*Gm(s)]
        
        However, in practice with python-control, we approximate the delay for the real plant
        (Gp(s) = Gm(s)*exp(-Ls)) using a Pade approximation or by a direct 'transport delay' if available.
        """
        # Approximate e^{-Ls} with Pade:
        delay_approx = ctrl.pade(self.delay, 1)  # 1st-order Pade for simplicity
        num_delay, den_delay = delay_approx
        
        # Real plant: Gp(s) = Gm(s) * e^{-Ls} (approx)
        Gp = ctrl.series(self.Gm, ctrl.TransferFunction(num_delay, den_delay))
        
        # Smith Predictor loop:
        # The closed-loop TF can be formed by the 'internal model control' approach:
        # T(s) = C(s)*Gp(s) / [1 + C(s)*(Gp(s) - Gm(s))].
        # But we also have the nominal loop in parallel. 
        # A simpler approach is to do it explicitly:
        
        # C(s)*Gp(s)
        CGp = ctrl.series(self.C, Gp)
        
        # Gp(s) - Gm(s)
        diff = ctrl.parallel(Gp, -self.Gm)
        
        # Denominator = 1 + C(s)*[Gp(s) - Gm(s)]
        denominator = ctrl.feedback(diff, self.C, sign=1)
        
        # Overall closed-loop (using "series / parallel" logic):
        closed_loop = ctrl.series(CGp, 1/denominator)
        
        return closed_loop
