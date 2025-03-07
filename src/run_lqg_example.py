import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

from controllers.lqg_controller import LQGController

def main():
    # 1. Define the continuous-time plant:
    #    x_dot = A x + B u + w   (process noise w)
    #    y     = C x + v        (measurement noise v)
    #
    # Let's use a 2-state system, e.g. a mass-spring-damper:
    #    x = [position, velocity]^T
    #
    # For a mass-spring-damper with mass m=1, damping c=0.2, spring k=1:
    #    [ x1_dot ] = [   0     1 ] [ x1 ] + [ 0 ] u
    #    [ x2_dot ]   [  -1   -0.2 ] [ x2 ]   [ 1 ]
    #
    # So A = [[0, 1],[ -1, -0.2]]
    #    B = [[0],[1]]
    #    C = [1, 0]  (we measure position only)

    A = np.array([[0.0, 1.0],
                  [-1.0, -0.2]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])  # measure position
    # G matrix for process noise can be same as B or identity, but let's keep it simple
    # G = B
    G = np.eye(2)  # shape (2,2)

    # 2. Design LQR gain K
    #    Suppose we want to penalize position heavily, velocity moderately, and control effort lightly.
    Q = np.diag([10.0, 1.0])  # state cost
    R = np.array([[0.1]])     # input cost
    K_lqr, S, E = ctrl.lqr(A, B, Q, R)
    # Note python-control returns K that is row vector if single input

    # 3. Design Kalman filter gain L (continuous-time LQE)
    #    We define process noise covariance Qn and measurement noise covariance Rn
    Qn = np.diag([0.1, 0.1])  # process noise on x1_dot, x2_dot
    Rn = np.array([[0.01]])   # measurement noise on y
    # L, P, E = ctrl.lqe(A, G, C, Qn, Rn)
    # python-control's lqe => returns L (the observer gain matrix).
    # But note: lqe uses (A, G, C, Qn, Rn).
    L_kf, P_e, E_e = ctrl.lqe(A, G, C, Qn, Rn)

    # 4. Create LQG controller
    #    We'll do a discrete simulation with dt=0.01 for demonstration.
    dt = 0.01
    lqg = LQGController(A, B, C, K_lqr, L_kf, dt)

    # 5. Simulate the closed-loop system with process and measurement noise in a loop
    t_final = 10.0
    t_vec = np.arange(0, t_final, dt)

    # We'll store arrays for true state x, measured output y, estimated state x_hat, and control
    x_data = np.zeros((len(t_vec), 2))
    x_hat_data = np.zeros((len(t_vec), 2))
    y_data = np.zeros(len(t_vec))
    u_data = np.zeros(len(t_vec))

    # Initialize x and x_hat
    x = np.array([[0.5],  # initial position
                  [0.0]]) # initial velocity
    lqg.reset(x_hat0=np.array([[0.0],[0.0]]))  # different from true state

    # Noise standard deviations
    w_std = 0.05  # process noise
    v_std = 0.01  # measurement noise

    rng = np.random.default_rng(42)  # for reproducibility

    # Let's assume a reference of 0 (we want to keep position near 0).
    # If you want to track a non-zero reference, you can augment the system or do integrator approach.

    for i, t in enumerate(t_vec):
        # measure y with noise
        y = C @ x + v_std*rng.normal(size=(1,1))
        y_data[i] = y.ravel()

        # LQG step: get control based on the measurement
        u = lqg.step(y)
        u_data[i] = u[0,0]

        # store current states
        x_data[i] = x.ravel()
        x_hat_data[i] = lqg.x_hat.ravel()

        # Plant update (Euler discretization)
        # x_dot = A x + B u + w, where w ~ N(0, w_std^2)
        w = w_std*rng.normal(size=(2,1))
        x_dot = A @ x + B @ u + w
        x = x + dt*x_dot

    # 6. Plot results
    plt.figure(figsize=(8,6))

    plt.subplot(3,1,1)
    plt.plot(t_vec, x_data[:,0], label='True Pos')
    plt.plot(t_vec, x_hat_data[:,0], '--', label='Est Pos')
    plt.ylabel("Position")
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(t_vec, x_data[:,1], label='True Vel')
    plt.plot(t_vec, x_hat_data[:,1], '--', label='Est Vel')
    plt.ylabel("Velocity")
    plt.legend()

    plt.subplot(3,1,3)
    plt.plot(t_vec, u_data, label='Control Input')
    plt.ylabel("u")
    plt.xlabel("Time (s)")
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
