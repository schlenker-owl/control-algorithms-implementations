import numpy as np
import matplotlib.pyplot as plt

from controllers.pid_controller import PIDController
from controllers.zn_tuning import ziegler_nichols_tuning
from data.synthetic_data import SyntheticDataGenerator

def main():
    # Suppose from an experiment you found Ku=10, Pu=2
    Ku, Pu = 10, 2

    Kp, Ki, Kd = ziegler_nichols_tuning(Ku, Pu, controller_type='pid')
    print(f"Ziegler-Nichols Tuning Results -> Kp: {Kp:.3f}, Ki: {Ki:.3f}, Kd: {Kd:.3f}")

    # 1. Setup
    gen = SyntheticDataGenerator()
    t, setpoint = gen.step_data(step_time=2.0, total_time=10.0, amplitude=1.0)
    dt = t[1] - t[0]

    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, dt=dt)

    # 2. Run simulation
    output = np.zeros_like(t)
    control_signal = np.zeros_like(t)

    for i in range(1, len(t)):
        measurement = output[i-1]
        control_signal[i] = pid.compute(setpoint[i], measurement)
        # Plant
        output[i] = output[i-1] + dt*(control_signal[i] - output[i-1])

    # 3. Plot
    plt.figure()
    plt.plot(t, setpoint, 'g--', label='Setpoint')
    plt.plot(t, output, 'b-', label='Output')
    plt.plot(t, control_signal, 'r-', label='Control')
    plt.legend()
    plt.title("Ziegler-Nichols Tuned PID")
    plt.show()

if __name__ == "__main__":
    main()
