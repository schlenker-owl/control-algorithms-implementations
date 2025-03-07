import matplotlib.pyplot as plt
import control as ctrl
import numpy as np

from controllers.smith_predictor import SmithPredictor

def main():
    # 1. Define a nominal plant (no-delay): Gm(s) = 1/(s+1)
    Gm = ctrl.tf([1], [1, 1])  # A simple first-order system

    # 2. Assume we have an actual time delay, say L = 1 second
    delay = 1.0

    # 3. Design a simple controller for Gm(s) ignoring delay
    #    For demonstration: let's do a simple PID or just P
    #    python-control can represent a P as a TransferFunction
    Kp = 2.0
    C = ctrl.tf([Kp], [1])  # P controller

    # 4. Create Smith Predictor object
    sp = SmithPredictor(controller_no_delay=C, plant_nominal=Gm, delay=delay)

    # 5. Construct the Smith predictor closed-loop
    T_smith = sp.closed_loop_tf()

    # For comparison: let's also see the closed-loop if we just do naive feedback
    # with the actual delayed plant: Gp(s) = Gm(s)*e^{-Ls} (pade approx)
    delay_approx = ctrl.pade(delay, 1)
    num_delay, den_delay = delay_approx
    Gp_approx = ctrl.series(Gm, ctrl.TransferFunction(num_delay, den_delay))
    T_naive = ctrl.feedback(ctrl.series(C, Gp_approx), 1)

    # 6. Step response
    t = np.linspace(0, 10, 300)
    t_smith, y_smith = ctrl.step_response(T_smith, t)
    t_naive, y_naive = ctrl.step_response(T_naive, t)

    # 7. Plot
    plt.figure()
    plt.plot(t_smith, y_smith, label='With Smith Predictor')
    plt.plot(t_naive, y_naive, label='Naive Delay Handling', linestyle='--')
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.title("Smith Predictor vs. Naive Control (Delay=1s)")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
