from visualise import Visualiser
from data import Sensor
from PyQt5 import QtCore, QtWidgets
import sys
import os
import pytest
import csv


class TestVisualise:
    @pytest.fixture(autouse=True)
    def run_before_test(self):
        f = lambda x: 5 * x
        sensor = Sensor(f, "Test sensor", "x", "y", step_size=0.01)
        self.sensors = [sensor]
        self.app = Visualiser(self.sensors)

    def test_start_recording(self, qtbot):
        qtbot.addWidget(self.app)
        # User presses the record button
        qtbot.mouseClick(self.app.button_record, QtCore.Qt.LeftButton)
        # Expect data to be added to recordings
        assert self.app.recording == True
        assert len(self.app.recordings) == len(self.sensors)
        self.app.update_graph()
        assert len(self.app.recordings[0]["x"]) > 0
        assert len(self.app.recordings[0]["y"]) > 0
        assert self.app.button_record.text() == '> Recording <'

    def test_stop_recording(self, qtbot):
        qtbot.addWidget(self.app)
        # User clicks the record button
        qtbot.mouseClick(self.app.button_record, QtCore.Qt.LeftButton)
        # User clicks the stop button
        qtbot.mouseClick(self.app.button_stop, QtCore.Qt.LeftButton)
        # Expect recording to stop
        assert self.app.recording == False
        assert self.app.button_record.text() == 'Record'

    def test_restart_recording(self, qtbot):
        qtbot.addWidget(self.app)
        # User clicks restart
        qtbot.mouseClick(self.app.button_restart, QtCore.Qt.LeftButton)
        # Expect stopped recording
        assert self.app.recording == False
        assert len(self.app.recordings) == len(self.sensors)
        # User presses the record button
        qtbot.mouseClick(self.app.button_record, QtCore.Qt.LeftButton)
        # User presses the restart button
        self.app.update_graph()
        qtbot.mouseClick(self.app.button_restart, QtCore.Qt.LeftButton)
        # Expect recording to be stopped and data cleared
        assert self.app.recording == False
        assert len(self.app.recordings[0]["x"]) == 0
        assert len(self.app.recordings[0]["y"]) == 0

    def test_save_recording(self, qtbot):
        qtbot.addWidget(self.app)
        # User clicks save with no recorded data
        def handle_dialog():
            ok_button = self.app.message_box.button(QtWidgets.QMessageBox.Ok)
            qtbot.mouseClick(ok_button, QtCore.Qt.LeftButton)

        QtCore.QTimer.singleShot(800, handle_dialog)
        qtbot.mouseClick(self.app.button_save, QtCore.Qt.LeftButton)

    def test_recording_to_csv(self, qtbot):
        self.app.start_record()
        self.app.update_graph()
        self.app.stop_record()
        test_file = "test.csv"
        self.app.recordings_to_csv(test_file)
        with open(test_file, newline="") as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                assert int(row[0]) == 0
                assert int(row[1]) == 0

        if os.path.exists(test_file):
            os.remove(test_file)
