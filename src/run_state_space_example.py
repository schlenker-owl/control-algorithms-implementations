import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

from controllers.state_space_controller import pole_placement, lqr_controller

def main():
    # 1. Define system in state-space
    #    e.g. A = [[0, 1],
    #              [0, -1]]
    #         B = [[0],
    #              [1]]
    A = np.array([[0, 1],
                  [0, -1]])
    B = np.array([[0],
                  [1]])
    C = np.array([[1, 0]])
    D = np.array([[0]])

    sys_ss = ctrl.ss(A, B, C, D)

    # 2. Pole Placement example
    desired_poles = [-2, -3]
    K = pole_placement(A, B, desired_poles)
    print("Pole-placement gain K =", K)

    # Form closed-loop system (assuming y(t)=x(t), u=-Kx)
    A_cl = A - B @ K
    sys_cl = ctrl.ss(A_cl, B, C, D)
    
    # Step response
    t, y = ctrl.step_response(sys_cl, T=5)
    
    plt.figure()
    plt.plot(t, y, label='Pole-Placement Response')
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.grid(True)
    plt.legend()
    plt.show()

    # 3. LQR example
    Q = np.diag([10, 1])
    R = np.array([[1]])
    K_lqr, S, E = lqr_controller(A, B, Q, R)
    print("LQR Gain K =", K_lqr)
    
    # New closed-loop
    A_cl_lqr = A - B @ K_lqr
    sys_cl_lqr = ctrl.ss(A_cl_lqr, B, C, D)

    t2, y2 = ctrl.step_response(sys_cl_lqr, T=5)
    plt.figure()
    plt.plot(t2, y2, label='LQR Response')
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
