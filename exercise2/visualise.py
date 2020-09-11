import math
from data import Sensor
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys


class Visualiser(QtWidgets.QMainWindow):
    """Visualise data from a sensor, requires sensor read() function."""

    def __init__(self, sensor: Sensor, *args, **kwargs) -> None:
        super(Visualiser, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground("w")
        self.setCentralWidget(self.graphWidget)
        self.pen = pg.mkPen(color=(255, 0, 0))

        self.sensor = sensor
        # one line for each measurement type
        self.lines = []
        for i in range(0, sensor.number_of_measurements):
            line = {
                'ref': self.graphWidget.plot([0], [0]),
                'x': [],
                'y': [],
            }
            self.lines.append(line)

        self.max_x = 200
        self.max_y = 200

        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_graph)
        self.timer.start()

    def update_graph(self) -> None:
        t, measurements = self.sensor.read()
       
        for i, line in enumerate(self.lines):
            line['x'].append(t)
            line['y'].append(measurements[i])
            # remove oldest value to keep size
            if len(line['x']) >= self.max_x: line['x'] = line['x'][1:]
            if len(line['y']) >= self.max_y: line['y'] = line['y'][1:]
                
            line['ref'].setData(line['x'], line['y'])



def main():
    sensor = Sensor()

    app = QtWidgets.QApplication(sys.argv)
    visualiser = Visualiser(sensor)
    visualiser.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
