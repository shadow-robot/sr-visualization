# Copyright 2011, 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
import sys
import os
import rospy
import rosnode
import rospkg
import threading

from QtCore import Qt, QTimer
from QtGui import QColor
from QtWidgets import (
    QWidget,
    QMessageBox,
    QFrame,
    QHBoxLayout,
    QCheckBox,
    QLabel,
    QFileDialog,
    QApplication,
    QVBoxLayout
)
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from sr_hand_health_report.monotonicity_check import MonotonicityCheck
from sr_hand_health_report.position_sensor_noise_check import PositionSensorNoiseCheck
from sr_hand_health_report.motor_check import MotorCheck

import queue
from datetime import datetime
import yaml
from os.path import exists


class SrHealthCheck(Plugin):

    """
    A GUI plugin for bootloading the motors on the shadow etherCAT hand.
    """

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('SrGuiHealthCheck')
        self._widget = QWidget()
        self._timer = QTimer()
        self._timer.timeout.connect(self.timerEvent)
        self._results = {}

        self._fingers = ['FF']
        self._side = "right"
        self._check_names = ['motor', 'position_sensor_noise']
        self._checks_to_execute = {}
        self._checks_running = False
        self._check_queue = queue.Queue()
        self._file_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_health_check'), 'uis', 'SrHealthCheck.ui')
        loadUi(ui_file, self._widget)
        self._widget.setLayout(QVBoxLayout())

        if context:
            context.add_widget(self._widget)

        self.initialize_checks()
        self.setup_connections()
        self._timer.start(100)

        threading.Thread(target=self.worker, daemon=True).start()

    def timerEvent(self):
        checks_are_running = False
        for check_name in self._check_names:
            if self._checks_to_execute[check_name]['thread'].is_alive():
                checks_are_running = True
        self._widget.button_start_selected.setEnabled(not checks_are_running)

    def setup_connections(self):
        self._widget.button_start_selected.clicked.connect(self.button_start_selected_clicked)
        self._widget.button_start_all.clicked.connect(self.button_start_all_clicked)

    def initialize_checks(self):
        self._file_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        for check_name in self._check_names:
            self._checks_to_execute[check_name] = {'check': 0, 'thread': 0}

        self._checks_to_execute['motor']['check'] = MotorCheck(self._side, self._fingers)
        self._checks_to_execute['position_sensor_noise']['check'] = PositionSensorNoiseCheck(self._side, self._fingers)

        self._checks_to_execute['motor']['thread'] = \
            threading.Thread(target=self._checks_to_execute['motor']['check'].run_check)
        self._checks_to_execute['position_sensor_noise']['thread'] = \
            threading.Thread(target=self._checks_to_execute['position_sensor_noise']['check'].run_check)

    def button_start_selected_clicked(self):
        self._file_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._file_name] = []

        rospy.logwarn(f"selected {self._file_name}")
        is_checked = {}
        is_checked['motor'] = self._widget.checkbox_motor.isChecked()
        is_checked['position_sensor_noise'] = self._widget.checkbox_position_sensor.isChecked()
        '''
        to be extended with more checks
        '''

        for check_name in self._check_names:
            if is_checked[check_name]:
                self._checks_to_execute[check_name]['thread'] = \
                    threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
                self._check_queue.put(self._checks_to_execute[check_name])

    def button_start_all_clicked(self):
        self._file_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._file_name] = []

        rospy.logwarn(f"all {self._file_name}")
        for check_name in self._check_names:
            self._checks_to_execute[check_name]['thread'] = \
                threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
            self._check_queue.put(self._checks_to_execute[check_name])

    def worker(self):
        while True:
            check = self._check_queue.get()
            check['thread'].start()
            check['thread'].join()

            self._check_queue.task_done()

            rospy.sleep(0.1)
            rospy.logwarn(f"{self._results}")
            rospy.logwarn(f"worker {self._file_name}")
            self._results[self._file_name].append(check['check'].get_result())

            rospy.logwarn(f"{self._check_queue.empty()}")
            if self._check_queue.empty():
                file = f"{rospkg.RosPack().get_path('sr_health_check')}/src/sr_health_check/results.yaml"
                current_data = None
                if exists(file):
                    with open(file, 'r', encoding="ASCII") as yaml_file:
                        current_data = yaml.safe_load(yaml_file)
                        rospy.logwarn(current_data)
                    with open(file, 'w', encoding="ASCII") as yaml_file:
                        current_data.update(self._results)
                        yaml.safe_dump(current_data, stream=yaml_file, default_flow_style=False)
                else:
                    with open(file, 'w', encoding="ASCII") as yaml_file:
                        yaml.safe_dump(self._results, stream=yaml_file, default_flow_style=False)
                self._results = {}

    def get_widget():
        return self._widget

    def shutdown_plugin(self):
        pass

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass


if __name__ == "__main__":
    rospy.init_node("sr_health_check_gui")
    app = QApplication(sys.argv)
    ctrl = SrHealthcheckGui(None)
    ctrl.get_widget().show()  # pylint: disable=W0212
    app.exec_()
