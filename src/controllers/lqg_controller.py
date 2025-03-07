import numpy as np

class LQGController:
    """
    LQG (Linear Quadratic Gaussian) controller for a continuous-time system:
      x_dot = A x + B u + w   (process noise w)
      y     = C x + v         (measurement noise v)

    Where the control input is u = -K * x_hat,
    and x_hat is estimated by a continuous-time Kalman filter:
      x_hat_dot = A x_hat + B u + L (y - C x_hat)

    For simplicity, we discretize the update equations with a small time step dt.
    """

    def __init__(self, A, B, C, K, L, dt):
        """
        :param A, B, C: System matrices (numpy arrays)
        :param K: LQR gain (numpy array)
        :param L: Kalman filter gain (numpy array)
        :param dt: Sampling time for discrete approximation
        """
        self.A = A
        self.B = B
        self.C = C
        self.K = K  # from LQR
        self.L = L  # from Kalman filter
        self.dt = dt

        # Number of states
        self.nx = A.shape[0]

        # Initialize state estimate
        self.x_hat = np.zeros((self.nx, 1))

    def reset(self, x_hat0=None):
        """ Reset the estimated state. """
        if x_hat0 is None:
            self.x_hat = np.zeros((self.nx, 1))
        else:
            self.x_hat = x_hat0

    def compute_control(self):
        """
        Compute control input based on current estimate x_hat.
        u = -K x_hat
        """
        u = -self.K @ self.x_hat
        return u

    def update_observer(self, y_meas, u):
        """
        Update the Kalman filter (observer) with measurement y_meas and input u.
        We'll do a simple Euler discretization for demonstration:
        
        x_hat_dot = A x_hat + B u + L (y_meas - C x_hat)
        x_hat[k+1] = x_hat[k] + dt * x_hat_dot
        """
        y_est = self.C @ self.x_hat
        innov = y_meas - y_est  # measurement residual
        x_hat_dot = self.A @ self.x_hat + self.B @ u + self.L @ innov
        self.x_hat = self.x_hat + self.dt * x_hat_dot

    def step(self, y_meas):
        """
        Convenience function that:
          1) computes control
          2) updates observer state
          3) returns control
        You can also do these in separate calls if you prefer.
        """
        u = self.compute_control()
        self.update_observer(y_meas, u)
        return u
