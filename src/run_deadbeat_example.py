import numpy as np
import matplotlib.pyplot as plt
from controllers.deadbeat_controller import DeadbeatController

def main():
    # 1. Assume a discrete first-order plant:
    #    x[k+1] = a*x[k] + b*u[k]
    #    Let's pick a=0.8, b=0.2 for demonstration
    a = 0.8
    b = 0.2

    # 2. Deadbeat controller (place next state at the setpoint in one step)
    db_controller = DeadbeatController(a, b)

    # 3. Simulation
    dt = 1.0  # discrete sampling interval (just for conceptual timeline)
    kmax = 20
    k_vec = np.arange(kmax)
    
    setpoint = np.zeros_like(k_vec, dtype=float)
    setpoint[k_vec >= 5] = 2.0  # step at k=5

    x = np.zeros_like(k_vec, dtype=float)
    u = np.zeros_like(k_vec, dtype=float)

    for k in range(kmax-1):
        # compute control
        u[k] = db_controller.compute(setpoint[k], x[k])
        # update plant
        x[k+1] = a * x[k] + b * u[k]

    # 4. Plot
    plt.figure()
    plt.step(k_vec, setpoint, 'g--', where='post', label='Setpoint')
    plt.step(k_vec, x, 'b-', where='post', label='Output (x[k])')
    plt.step(k_vec, u, 'r-', where='post', label='Control (u[k])')
    plt.title("Deadbeat Controller (Discrete-Time)")
    plt.xlabel("Sample k")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
