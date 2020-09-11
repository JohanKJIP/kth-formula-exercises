import math


class Sensor:
    """Imaginary sensor. Produces values for velocity and acceleration."""

    def __init__(self, step_size: int = 0.001) -> None:
        self.step_size = step_size
        self.eta = 1e-6
        self.step = 0

    def read(self) -> None:
        """ Read current sensor values. """
        lamda = lambda t: 5 * math.sin(2 * math.pi * t)
        h = lambda t: 3 * math.pi * math.exp(-lamda(t))

        velocity = h(self.step)
        # numerical differentiation
        acceleration = (h(self.step + self.eta) - h(self.step - self.eta)) / (
            2 * self.eta
        )

        self.step += self.step_size
        return self.step, velocity, acceleration
