import numpy as np

class SyntheticDataGenerator:
    """
    Class for generating synthetic data for testing control algorithms.
    """

    def __init__(self, seed=None):
        """
        Initialize the data generator.

        :param seed: Optional seed for reproducibility.
        """
        self.rng = np.random.default_rng(seed)

    def step_data(self, step_time=1.0, total_time=10.0, dt=0.01, amplitude=1.0):
        """
        Generate a step input: 0 until step_time, then amplitude afterwards.
        
        :param step_time: Time (seconds) when step occurs.
        :param total_time: Total simulation time (seconds).
        :param dt: Time step.
        :param amplitude: Step amplitude.
        :return: t (time array), u (input array)
        """
        t = np.arange(0, total_time, dt)
        u = np.zeros_like(t)
        u[t >= step_time] = amplitude
        return t, u

    def sine_data(self, freq=1.0, total_time=10.0, dt=0.01, amplitude=1.0, phase=0.0):
        """
        Generate a sine wave.

        :param freq: Frequency of sine wave in Hz.
        :param total_time: Total simulation time (seconds).
        :param dt: Time step.
        :param amplitude: Amplitude of the sine.
        :param phase: Phase offset (radians).
        :return: t (time array), u (input array)
        """
        t = np.arange(0, total_time, dt)
        u = amplitude * np.sin(2 * np.pi * freq * t + phase)
        return t, u

    def random_data(self, total_time=10.0, dt=0.01, mean=0.0, std=1.0):
        """
        Generate random (Gaussian) data.

        :param total_time: Total simulation time (seconds).
        :param dt: Time step.
        :param mean: Mean of the random process.
        :param std: Standard deviation of the random process.
        :return: t (time array), u (input array)
        """
        t = np.arange(0, total_time, dt)
        u = self.rng.normal(loc=mean, scale=std, size=len(t))
        return t, u
