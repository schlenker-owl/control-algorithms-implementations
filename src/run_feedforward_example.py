import numpy as np
import matplotlib.pyplot as plt

from controllers.pid_controller import PIDController
from controllers.feedforward_controller import FeedForwardController

def main():
    # 1. Define a simple plant: y'(t) = -y(t) + u(t)
    #    If we try to invert it, feedforward_gain ~ 1.0 (since input ~ output + derivative)
    #    This is obviously oversimplified, but it’ll illustrate the concept.

    total_time = 10.0
    dt = 0.01
    t = np.arange(0, total_time, dt)

    setpoint = np.zeros_like(t)
    # Let’s do a step at t=2, amplitude=2
    setpoint[t >= 2.0] = 2.0

    # 2. Create a PID and a feed-forward controller
    pid = PIDController(Kp=0.5, Ki=0.1, Kd=0.0, dt=dt)
    # Suppose feedforward_gain = 1.0
    ff_ctrl = FeedForwardController(pid_controller=pid, feedforward_gain=1.0)

    # 3. Simulation
    y = np.zeros_like(t)
    u = np.zeros_like(t)

    for i in range(1, len(t)):
        measurement = y[i-1]
        u[i] = ff_ctrl.compute(setpoint[i], measurement)

        # Plant update: y'(t) = -y + u
        y[i] = y[i-1] + dt * ( -y[i-1] + u[i] )

    # 4. Plot
    plt.figure()
    plt.plot(t, setpoint, 'g--', label='Setpoint')
    plt.plot(t, y, 'b-', label='Output')
    plt.plot(t, u, 'r-', label='Control Input (u)')
    plt.title("Feed-Forward + PID Example")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
