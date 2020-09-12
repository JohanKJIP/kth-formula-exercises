import math
from typing import Callable


class Sensor:
    """Imaginary sensor. Produces values according to callable function."""

    def __init__(
        self,
        function: Callable,
        name: str,
        time_unit: str,
        value_unit: str,
        step_size: int = 0.01,
    ) -> None:
        self.name = name
        self.time_unit = time_unit
        self.value_unit = value_unit
        self.step_size = step_size
        self.step = 0
        self.f = function

    def read(self) -> None:
        """ Read current sensor values. """
        value = self.f(self.step)
        time = self.step

        self.step += self.step_size
        return time, value
