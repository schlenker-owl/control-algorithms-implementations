import numpy as np
import matplotlib.pyplot as plt

from controllers.pd_controller import PDController
from data.synthetic_data import SyntheticDataGenerator

def simulate_second_order_plant(u, dt, omega_n=2.0, zeta=0.5):
    """
    Example second-order system: y'' + 2*zeta*omega_n*y' + omega_n^2 * y = omega_n^2 * u
    Discretized using Euler's method for demonstration.
    """
    y = np.zeros_like(u)
    y_dot = np.zeros_like(u)

    for i in range(1, len(u)):
        # The continuous system is:
        # y''(t) = omega_n^2 * u(t) - 2*zeta*omega_n*y'(t) - omega_n^2*y(t)
        y_ddot = omega_n**2 * u[i-1] - 2*zeta*omega_n*y_dot[i-1] - omega_n**2*y[i-1]

        y_dot[i] = y_dot[i-1] + dt * y_ddot
        y[i] = y[i-1] + dt * y_dot[i-1]
    
    return y

def main():
    # 1. Generate a step setpoint
    gen = SyntheticDataGenerator()
    t, setpoint = gen.step_data(step_time=1.0, total_time=5.0, amplitude=2.0)
    dt = t[1] - t[0]

    # 2. Create PD controller
    pd = PDController(Kp=5.0, Kd=1.0, dt=dt)

    # 3. Simulation loop
    output = np.zeros_like(t)
    control_signal = np.zeros_like(t)

    for i in range(1, len(t)):
        measurement = output[i-1]
        control_signal[i] = pd.compute(setpoint[i], measurement)
        
        # Simulate second-order plant
        # We'll just treat the previous control signal as the system input for this time step
        # so we need to re-run the second-order eqn in incremental form
        # For clarity, let's do one step at a time:
        y_ddot = (2.0**2)*control_signal[i-1] - 2*0.5*2.0*(output[i-1]-output[i-2] if i>1 else 0) - (2.0**2)*output[i-1]
        # But to keep it simpler, let's just do the function approach:
        # We'll do a quick hack: re-run the entire simulation from 0..i each iteration
        # This is not very efficient for large i, but it’s okay for demo

    # More efficient approach: directly track the state
    output = simulate_second_order_plant(control_signal, dt)

    # 4. Plot results
    plt.figure()
    plt.plot(t, setpoint, 'g--', label='Setpoint')
    plt.plot(t, output, 'b-', label='Plant Output')
    plt.plot(t, control_signal, 'r-', label='Control Input')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Value')
    plt.title('PD Controller Example')
    plt.show()

if __name__ == "__main__":
    main()
