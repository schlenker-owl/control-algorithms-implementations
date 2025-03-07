import control as ctrl

class IMCController:
    """
    Internal Model Control (IMC) approach.
    
    For a plant G(s) = G_good(s)*G_bad(s), we define:
       Q(s) = (G_good(s)^-1) * F(s),
    where F(s) is a low-pass filter for robustness.
    
    The IMC Controller can be converted to a standard feedback controller if desired.
    """

    def __init__(self, plant_nominal, filter_tf):
        """
        :param plant_nominal: The invertible portion of the plant, G_good(s).
        :param filter_tf: A low-pass filter F(s), e.g., 1/(lambda*s + 1), etc.
        """
        self.Gm = plant_nominal
        self.F = filter_tf

        # Attempt to invert Gm(s), if possible
        self.Gm_inv = ctrl.minreal(1 / self.Gm)

        # IMC Q(s) = Gm_inv * F(s)
        self.Q = ctrl.minreal(ctrl.series(self.Gm_inv, self.F))

    def imc_closed_loop_tf(self, real_plant):
        """
        The internal IMC structure closed-loop transfer function with 
        real_plant (which might be G_good(s)*G_bad(s)).
        
        For reference, T(s) = [ Q(s)*G(s) ] / [1 + Q(s)*[G(s) - Gm(s)] ] in typical form.
        We'll do it in python-control using block manipulations.
        """
        # Q(s)*G(s)
        QG = ctrl.series(self.Q, real_plant)

        # G(s) - Gm(s)
        diff = ctrl.parallel(real_plant, -self.Gm)

        # python-control represents TF(0) as TransferFunction([0], [1])
        if diff.num[0][0] == [0] and diff.den[0][0] == [1]:
            # G(s) - Gm(s) == 0
            denominator = ctrl.tf([1], [1])
        else:
            denominator = ctrl.feedback(diff, self.Q, sign=1)

        # Overall closed-loop
        closed_loop = ctrl.series(QG, 1/denominator)
        return ctrl.minreal(closed_loop)
