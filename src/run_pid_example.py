import numpy as np
import matplotlib.pyplot as plt

from data.synthetic_data import SyntheticDataGenerator
from controllers.pid_controller import PIDController

def simulate_first_order_system(u, dt, tau=1.0, gain=1.0):
    """
    Simulate a simple first-order system: y'(t) + 1/tau * y(t) = gain * u(t)
    Discretized with Euler approximation for illustration purposes.
    """
    y = np.zeros_like(u)
    alpha = dt / tau

    for i in range(1, len(u)):
        # Euler integration
        y[i] = y[i-1] + alpha * (gain * u[i-1] - y[i-1])
    
    return y

def main():
    # 1. Generate synthetic data
    gen = SyntheticDataGenerator(seed=42)
    t, setpoint = gen.step_data(step_time=2.0, total_time=10.0, amplitude=1.0)

    dt = t[1] - t[0]
    
    # 2. Create PID controller
    pid = PIDController(Kp=2.0, Ki=1.0, Kd=0.5, dt=dt)

    # 3. Simulation loop
    measured_output = np.zeros_like(t)
    control_signal = np.zeros_like(t)

    for i in range(1, len(t)):
        # Compute control signal
        control_signal[i] = pid.compute(setpoint[i], measured_output[i-1])
        
        # 'Plant' or 'process' - a simple first-order system
        measured_output[i] = measured_output[i-1] + (dt/1.0)*(control_signal[i-1] - measured_output[i-1])

    # 4. Plot results
    plt.figure()
    plt.plot(t, setpoint, 'g--', label='Setpoint')
    plt.plot(t, measured_output, 'b-', label='Output')
    plt.plot(t, control_signal, 'r-', label='Control Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.title('PID Control Example')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
