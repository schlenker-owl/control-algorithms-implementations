import numpy as np
import matplotlib.pyplot as plt
from controllers.onoff_controller import OnOffController

def main():
    dt = 0.01
    t_max = 10.0
    t = np.arange(0, t_max, dt)

    # Let's track temperature in a "heater" scenario:
    # temp'(t) = -k*(temp - ambient) + heater_power * control
    # We'll do a simple numeric integration
    ambient_temp = 20.0
    k = 0.1
    heater_power = 1.0
    setpoint = 50.0

    # OnOff with hysteresis
    # E.g. turn on if error > 1 deg, turn off if error < 0 deg
    onoff = OnOffController(on_threshold=1.0, off_threshold=0.0)

    temp = np.zeros_like(t)
    temp[0] = 25.0  # initial temperature
    control_signal = np.zeros_like(t)

    for i in range(1, len(t)):
        measurement = temp[i-1]
        u = onoff.compute(setpoint, measurement)
        control_signal[i] = u

        dtemp = dt * (-k*(measurement - ambient_temp) + heater_power * u)
        temp[i] = temp[i-1] + dtemp

    # Plot
    plt.figure()
    plt.plot(t, temp, label='Temperature')
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.plot(t, control_signal*10 + ambient_temp, label='On/Off (scaled)')
    plt.title("On-Off (Bang-Bang) Control Example")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (C)")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
