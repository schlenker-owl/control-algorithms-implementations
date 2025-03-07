import control as ctrl

def lead_compensator(K, tau, alpha):
    """
    Continuous-time lead compensator:
      G_lead(s) = K * (tau s + 1) / (alpha * tau s + 1)
    Returns a python-control TransferFunction object.
    """
    num = [K*tau, K]
    den = [alpha*tau, 1]
    return ctrl.TransferFunction(num, den)

def lag_compensator(K, tau, beta):
    """
    Continuous-time lag compensator:
      G_lag(s) = K * (tau s + 1) / (tau s + beta)
    """
    num = [K*tau, K]
    den = [tau, beta]
    return ctrl.TransferFunction(num, den)
