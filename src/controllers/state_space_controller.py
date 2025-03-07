import control as ctrl
import numpy as np

def pole_placement(A, B, desired_poles):
    """
    Returns the state-feedback gain K given system matrices A, B.
    """
    K = ctrl.place(A, B, desired_poles)
    return K

def lqr_controller(A, B, Q, R):
    """
    Returns LQR gain K, cost-to-go S, closed-loop eigenvalues E.
    """
    K, S, E = ctrl.lqr(A, B, Q, R)
    return K, S, E
