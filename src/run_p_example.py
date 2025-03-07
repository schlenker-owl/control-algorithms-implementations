import numpy as np
import matplotlib.pyplot as plt

from controllers.p_controller import PController
from data.synthetic_data import SyntheticDataGenerator

def simulate_first_order_plant(u, dt, tau=1.0):
    """
    Simple first-order system: y'(t) = -1/tau * y(t) + 1/tau * u(t).
    Euler discretization for demonstration.
    """
    y = np.zeros_like(u)
    alpha = dt / tau
    for i in range(1, len(u)):
        y[i] = y[i-1] + alpha * (u[i-1] - y[i-1])
    return y

def main():
    # 1. Generate a step setpoint using SyntheticDataGenerator
    gen = SyntheticDataGenerator(seed=42)
    t, setpoint = gen.step_data(step_time=2.0, total_time=10.0, amplitude=1.0)
    dt = t[1] - t[0]

    # 2. Create P controller
    p = PController(Kp=2.0)

    # 3. Simulation loop
    output = np.zeros_like(t)
    control_signal = np.zeros_like(t)

    for i in range(1, len(t)):
        # Current measurement is the plant output
        measurement = output[i-1]

        # Controller calculates control signal
        control_signal[i] = p.compute(setpoint[i], measurement)

        # Plant update
        output[i] = output[i-1] + (dt/1.0) * (control_signal[i-1] - output[i-1])

    # 4. Plot results
    plt.figure()
    plt.plot(t, setpoint, 'g--', label='Setpoint')
    plt.plot(t, output, 'b-', label='Plant Output')
    plt.plot(t, control_signal, 'r-', label='Control Input')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Value')
    plt.title('Proportional Controller Example')
    plt.show()

if __name__ == "__main__":
    main()
