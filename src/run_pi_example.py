import numpy as np
import matplotlib.pyplot as plt

from controllers.pi_controller import PIController
from data.synthetic_data import SyntheticDataGenerator

def main():
    # 1. Generate step data
    gen = SyntheticDataGenerator()
    t, setpoint = gen.step_data(step_time=0.0, total_time=10.0, amplitude=1.0)
    dt = t[1] - t[0]

    # 2. Create PI Controller
    pi = PIController(Kp=2.0, Ki=1.0, dt=dt)

    # 3. Simple first-order plant simulation
    output = np.zeros_like(t)
    control_signal = np.zeros_like(t)

    for i in range(1, len(t)):
        measurement = output[i-1]
        control_signal[i] = pi.compute(setpoint[i], measurement)
        
        # Plant: y'(t) = control - y(t)
        output[i] = output[i-1] + dt*(control_signal[i-1] - output[i-1])

    # 4. Plot
    plt.figure()
    plt.plot(t, setpoint, 'g--', label='Setpoint')
    plt.plot(t, output, 'b-', label='Plant Output')
    plt.plot(t, control_signal, 'r-', label='Control Input')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Value')
    plt.title('PI Controller Example')
    plt.show()

if __name__ == "__main__":
    main()
