import numpy as np
import matplotlib.pyplot as plt
from controllers.gain_scheduling_controller import GainSchedulingPID

def main():
    dt = 0.01
    t_end = 10.0
    t = np.arange(0, t_end, dt)

    # We'll use a silly example: The "scheduling variable" is the
    # measured output itself. Suppose the system changes behavior
    # as output changes.

    # Create the gain-scheduled PID
    gs_pid = GainSchedulingPID(dt=dt)

    # Add some breakpoints: (sched_value, Kp, Ki, Kd)
    gs_pid.add_gain_point(sched_value=0.0, Kp=2.0, Ki=1.0, Kd=0.0)
    gs_pid.add_gain_point(sched_value=3.0, Kp=1.0, Ki=0.5, Kd=0.1)
    gs_pid.add_gain_point(sched_value=6.0, Kp=0.5, Ki=0.2, Kd=0.2)
    # beyond 6.0, it stays at last gains

    # We want to track a setpoint of 5
    setpoint = np.ones_like(t)*5.0

    # Simple plant: y'(t) = -0.5*y(t) + 0.5*u(t)
    y = np.zeros_like(t)
    u = np.zeros_like(t)

    for i in range(1, len(t)):
        measurement = y[i-1]
        # scheduling param is the current measurement for demonstration
        u[i] = gs_pid.compute(setpoint[i], measurement, sched_value=measurement)

        # update plant
        y[i] = y[i-1] + dt*( -0.5*y[i-1] + 0.5*u[i] )

    # Plot
    plt.figure()
    plt.plot(t, setpoint, 'g--', label='Setpoint')
    plt.plot(t, y, 'b-', label='Output')
    plt.plot(t, u, 'r-', label='Control Input')
    plt.legend()
    plt.title("Gain-Scheduling PID Example")
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.show()

if __name__ == "__main__":
    main()
