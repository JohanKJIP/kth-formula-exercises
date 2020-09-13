from data import Sensor


class TestData:
    """Tests for data.py"""

    def setup(self):
        self.name = "test sensor"
        self.time_unit = "x"
        self.value_unit = "y"

    def dummy_sensor(self):
        f = lambda x: 5 * x
        return Sensor(f, self.name, self.time_unit, self.value_unit, step_size=0.01)

    def test_member_values(self):
        sensor = self.dummy_sensor()
        assert sensor.name == self.name
        assert sensor.time_unit == self.time_unit
        assert sensor.value_unit == self.value_unit

    def test_read(self):
        sensor = self.dummy_sensor()
        t, y = sensor.read()
        assert t == 0
        assert y == sensor.f(t)

        t1, y1 = sensor.read()
        assert t1 == 0.01
        assert y1 == 0.01 * 5
