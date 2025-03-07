import numpy as np
import matplotlib.pyplot as plt

from controllers.cascade_controller import InnerPID, OuterPID

def main():
    # Imagine a scenario:
    # - The "inner loop" controls flow (fast)
    # - The "outer loop" controls temperature (slow), 
    #   by adjusting the flow setpoint.
    #
    # For demonstration, let's do a simplified approach:
    # We'll treat the flow as a first-order system
    # and the temperature as a slower first-order system
    # that depends on flow.

    total_time = 20.0
    dt = 0.01
    t = np.arange(0, total_time, dt)

    # Outer loop target (temperature setpoint)
    temp_setpoint = 50.0  # e.g. 50 degC
    # The outer loop will compute a "desired flow" based on temperature error

    # Create controllers
    outer_controller = OuterPID(Kp=0.5, Ki=0.1, Kd=0.0, dt=dt)
    inner_controller = InnerPID(Kp=2.0, Ki=1.0, Kd=0.0, dt=dt)

    # States
    temperature = np.zeros_like(t)
    flow = np.zeros_like(t)
    flow_setpoint = np.zeros_like(t)

    # Simple dynamic models (very rough)
    # Flow dynamics: flow'(t) = -1/Tf * flow(t) + 1/Tf * flow_control
    # Temperature depends on flow: temp'(t) = -1/Tt * temp(t) + K * flow(t)
    Tf = 0.5   # time constant for flow
    Tt = 5.0   # time constant for temperature
    Ktemp = 1.0  # scaling

    for i in range(1, len(t)):
        dt_i = dt

        # Outer loop calculates flow setpoint based on temperature error
        flow_setpoint[i] = outer_controller.compute(temp_setpoint, temperature[i-1])

        # Inner loop calculates control signal to achieve flow_setpoint
        # For simplicity, let's treat that control signal as "valve position" or something
        valve_control = inner_controller.compute(flow_setpoint[i], flow[i-1])

        # Update flow with first-order approximation
        flow[i] = flow[i-1] + dt_i * (1.0/Tf)*(valve_control - flow[i-1])

        # Update temperature with first-order approximation
        temperature[i] = temperature[i-1] + dt_i * (1.0/Tt)*(Ktemp*flow[i-1] - temperature[i-1])

    # Plot results
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(t, temperature, label='Temperature')
    plt.axhline(temp_setpoint, color='r', linestyle='--', label='Temp Setpoint')
    plt.title("Cascade Control Example (Temperature -> Flow)")
    plt.ylabel("Temperature")
    plt.legend()

    plt.subplot(2,1,2)
    plt.plot(t, flow, label='Flow')
    plt.plot(t, flow_setpoint, '--', label='Flow Setpoint')
    plt.xlabel("Time (s)")
    plt.ylabel("Flow")
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
