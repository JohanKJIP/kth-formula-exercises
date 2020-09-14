import math
from data import Sensor
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QCursor
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import csv
import random
import sys


class Visualiser(QtWidgets.QMainWindow):
    """Visualise data from sensors, requires sensor read() function."""

    def __init__(self, sensors: [Sensor], *args, **kwargs) -> None:
        super(Visualiser, self).__init__(*args, **kwargs)
        pg.setConfigOptions(antialias=True)

        self.central_widget = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        self.sensors = sensors

        # Add graphs to layout
        self.lines = []
        for sensor in sensors:
            self.graph_widget = pg.PlotWidget(title=sensor.name)
            self.graph_widget.setBackground("w")
            self.graph_widget.setLabel("left", sensor.value_unit)
            self.graph_widget.setLabel("bottom", sensor.time_unit)
            self.layout.addWidget(self.graph_widget)

            pen = pg.mkPen(
                color=(
                    random.randint(0, 255),
                    random.randint(0, 255),
                    random.randint(0, 255),
                ),
                width=2,
            )
            line = {
                "ref": self.graph_widget.plot([0], [0], pen=pen),
                "x": [],
                "y": [],
            }
            self.lines.append(line)

        # Add buttons to layout
        self.buttons_row = QtWidgets.QHBoxLayout()
        self.layout.addLayout(self.buttons_row)

        self.button_record = QtWidgets.QPushButton("Record")
        self.buttons_row.addWidget(self.button_record)
        self.button_record.clicked.connect(self.start_record)
        self.button_record.setCursor(QCursor(QtCore.Qt.PointingHandCursor))

        self.button_stop = QtWidgets.QPushButton("Stop")
        self.buttons_row.addWidget(self.button_stop)
        self.button_stop.clicked.connect(self.stop_record)
        self.button_stop.setCursor(QCursor(QtCore.Qt.PointingHandCursor))

        self.button_restart = QtWidgets.QPushButton("Restart")
        self.buttons_row.addWidget(self.button_restart)
        self.button_restart.clicked.connect(self.restart_record)
        self.button_restart.setCursor(QCursor(QtCore.Qt.PointingHandCursor))

        self.buttons_row.addSpacing(200)

        self.button_save = QtWidgets.QPushButton("Save")
        self.buttons_row.addWidget(self.button_save)
        self.button_save.clicked.connect(self.save_record)
        self.button_save.setCursor(QCursor(QtCore.Qt.PointingHandCursor))

        self.max_x = 1200
        self.max_y = 1200
        self.recording = False
        self.recordings = []

        self.timer = QtCore.QTimer()
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.update_graph)
        self.timer.start()

    def restart_record(self) -> None:
        """Stop and restart the recording."""
        self.stop_record()
        self.recordings = []
        for sensor in self.sensors:
            data_recording = {"x": [], "y": []}
            self.recordings.append(data_recording)
    
    def start_record(self) -> None:
        """Start recording data from sensors."""
        if len(self.recordings) == 0:
            self.restart_record()
        self.recording = True
        self.button_record.setStyleSheet('QPushButton {background-color: #A3C1DA}')
        self.button_record.setText('> Recording <')

    def stop_record(self) -> None:
        """Stop recording data from sensors."""
        self.recording = False
        self.button_record.setStyleSheet('QPushButton {background-color: light gray}')
        self.button_record.setText('Record')

    def save_record(self) -> None:
        """Save recorded data from sensors to csv file."""
        self.stop_record()
        if len(self.recordings) > 0 and len(self.recordings[0]['x']) > 0:
            options = QtWidgets.QFileDialog.Options()
            options |= QtWidgets.QFileDialog.DontUseNativeDialog
            file_name, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                "QFileDialog.getSaveFileName()",
                "",
                "All Files (*);;Text Files (*.txt)",
                options=options,
            )
            if file_name:
                self.recordings_to_csv(file_name)
        else:
            self.message_box = QtWidgets.QMessageBox()
            self.message_box.setIcon(QtWidgets.QMessageBox.Information)
            self.message_box.setText("Problem saving: no data recorded.")
            self.message_box.exec()

    def recordings_to_csv(self, path: str) -> None:
        """Save recordings to a csv file"""
        with open(path, 'w', newline="") as f:
            writer = csv.writer(f)
            # csv can only write rows
            for i in range(0, len(self.recordings[0]['x'])):
                row = []
                for recording in self.recordings:
                    row.append(recording['x'][i])
                    row.append(recording['y'][i])
                writer.writerow(row)


    def update_graph(self) -> None:
        """Update graph and record values."""
        for i, (sensor, line) in enumerate(zip(self.sensors, self.lines)):
            t, value = sensor.read()
            line["x"].append(t)
            line["y"].append(value)
            # remove oldest value to keep size
            if len(line["x"]) >= self.max_x:
                line["x"] = line["x"][1:]
            if len(line["y"]) >= self.max_y:
                line["y"] = line["y"][1:]
            line["ref"].setData(line["x"], line["y"])

            if self.recording:
                self.recordings[i]["x"].append(t)
                self.recordings[i]["y"].append(value)


def main():
    simulation_step_size = 0.001

    # imaginary sensor following the given function
    lamda = lambda t: 5 * math.sin(2 * math.pi * t)
    h = lambda t: 3 * math.pi * math.exp(-lamda(t))
    velocity_sensor = Sensor(
        h,
        "Velocity sensor",
        f"step ({simulation_step_size})",
        "velocity (m/s)",
        step_size=simulation_step_size,
    )

    epsilon = 1e-8
    # numerical derivation of h
    dh = lambda t: (h(t + epsilon) - h(t - epsilon)) / (2 * epsilon)
    acceleration_sensor = Sensor(
        dh,
        "Acceleration sensor",
        f"step ({simulation_step_size})",
        "acceleration (m/s^2)",
        step_size=simulation_step_size,
    )

    constant = lambda t: 500
    constant_sensor = Sensor(
        constant,
        "Constant sensor",
        f"step ({simulation_step_size})",
        "Units",
        step_size=simulation_step_size,
    )

    sensors = [velocity_sensor, acceleration_sensor, constant_sensor]

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Visualiser")
    visualiser = Visualiser(sensors)
    visualiser.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
