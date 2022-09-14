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
from collections import OrderedDict


class SrHealthCheck(Plugin):

    """
    A GUI plugin for health check execution.
    """

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('SrGuiHealthCheck')
        self._timer = QTimer()
        self._timer.timeout.connect(self.timerEvent)
        self._results = OrderedDict()

        self._fingers = ['FF']  # to be parametrized
        self._side = "right"  # to be parametrized
        self._check_names = ['motor', 'position_sensor', 'monotonicity']  # to be extended
        self._checks_to_execute = {}
        self._checks_running = False
        self._check_queue = queue.Queue()

        self._current_data = None
        self._currently_selected_check = dict.fromkeys(self._check_names, False)
        self._filename = None

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('sr_health_check'), 'uis', 'SrHealthCheck.ui')
        loadUi(ui_file, self._widget)
        self._widget.setLayout(QVBoxLayout())

        if context:
            context.add_widget(self._widget)

        self.initialize_checks()
        self.setup_connections()
        self._timer.start(100)

    def timerEvent(self):
        checks_are_running = False
        for check_name in self._check_names:
            if self._checks_to_execute[check_name]['thread'].is_alive():
                checks_are_running = True
        selected_checks = any(check for check in self._check_names if self._currently_selected_check[check])
        self._widget.button_start_selected.setEnabled(not checks_are_running and selected_checks)
        self._widget.button_start_all.setEnabled(not checks_are_running)
        self._widget.button_details_motor.setEnabled(self._widget.passed_motor.text() != "-")
        self._widget.button_details_position_sensor.setEnabled(self._widget.passed_position_sensor.text() != "-")
        self._widget.button_details_monotonicity.setEnabled(self._widget.passed_monotonicity.text() != "-")

    def initialize_checks(self):
        for check_name in self._check_names:
            self._checks_to_execute[check_name] = {'check': 0, 'thread': 0}

        self._checks_to_execute['motor']['check'] = MotorCheck(self._side, self._fingers)
        self._checks_to_execute['position_sensor']['check'] = PositionSensorNoiseCheck(self._side, self._fingers)
        self._checks_to_execute['monotonicity']['check'] = MonotonicityCheck(self._side, self._fingers)

        self._checks_to_execute['motor']['thread'] = \
            threading.Thread(target=self._checks_to_execute['motor']['check'].run_check)
        self._checks_to_execute['position_sensor']['thread'] = \
            threading.Thread(target=self._checks_to_execute['position_sensor']['check'].run_check)
        self._checks_to_execute['monotonicity']['thread'] = \
            threading.Thread(target=self._checks_to_execute['monotonicity']['check'].run_check)

    def setup_connections(self):
        self._widget.button_start_selected.clicked.connect(self.button_start_selected_clicked)
        self._widget.button_start_all.clicked.connect(self.button_start_all_clicked)
        self._widget.button_details_motor.clicked.connect(self.button_details_clicked)
        self._widget.button_details_position_sensor.clicked.connect(self.button_details_clicked)
        self._widget.button_details_monotonicity.clicked.connect(self.button_details_clicked)

        self._widget.checkbox_motor.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_position_sensor.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_monotonicity.clicked.connect(self.checkbox_selected)

        threading.Thread(target=self.check_execution, daemon=True).start()

    def button_start_selected_clicked(self):
        self._filename = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._filename] = []
        for check_name in self._check_names:
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
            if self._currently_selected_check[check_name]:
                self._checks_to_execute[check_name]['thread'] = \
                    threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
                self._check_queue.put(self._checks_to_execute[check_name])

    def button_start_all_clicked(self):
        self._filename = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._filename] = []
        for check_name in self._check_names:
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
            self._checks_to_execute[check_name]['thread'] = \
                threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
            self._check_queue.put(self._checks_to_execute[check_name])

    def button_details_clicked(self):
        caller = self.sender()
        check_name = caller.accessibleName()
        message = str(self._checks_to_execute[check_name]['check'].get_result())
        msg = QMessageBox()
        msg.setWindowTitle(f"{check_name.capitalize()} check details")
        msg.setIcon(QMessageBox().Information)
        msg.setText(message)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def checkbox_selected(self):
        self._currently_selected_check['motor'] = self._widget.checkbox_motor.isChecked()
        self._currently_selected_check['position_sensor'] = self._widget.checkbox_position_sensor.isChecked()
        self._currently_selected_check['monotonicity'] = self._widget.checkbox_monotonicity.isChecked()

    def check_execution(self):
        while True:
            check = self._check_queue.get()
            check['thread'].start()
            self.update_passed_label(check['check'], "Executing")
            check['thread'].join()

            self._check_queue.task_done()
            self._results[self._filename].append(check['check'].get_result())
            self.update_passed_label(check['check'])

            if self._check_queue.empty():
                file = f"{rospkg.RosPack().get_path('sr_health_check')}/src/sr_health_check/results.yaml"
                if exists(file):
                    with open(file, 'r', encoding="ASCII") as yaml_file:
                        self._current_data = yaml.safe_load(yaml_file)
                    with open(file, 'w', encoding="ASCII") as yaml_file:
                        if self._current_data:
                            self._current_data.update(self._results)
                        else:
                            self._current_data = dict(self._results)
                        yaml.safe_dump(self._current_data, stream=yaml_file, default_flow_style=False)
                else:
                    with open(file, 'w', encoding="ASCII") as yaml_file:
                        self._current_data = dict(self._results)
                        yaml.safe_dump(self._current_data, stream=yaml_file, default_flow_style=False)

    def update_passed_label(self, check, text=None):
        if not text:
            if isinstance(check, MotorCheck):
                self._widget.passed_motor.setText(str(self._checks_to_execute['motor']['check'].has_passed()))
            elif isinstance(check, PositionSensorNoiseCheck):
                self._widget.passed_position_sensor.setText(str(self._checks_to_execute['position_sensor']
                                                                ['check'].has_passed()))
            elif isinstance(check, MonotonicityCheck):
                self._widget.passed_monotonicity.setText(str(self._checks_to_execute['monotonicity']
                                                             ['check'].has_passed()))
        else:
            if isinstance(check, MotorCheck):
                self._widget.passed_motor.setText(text)
            elif isinstance(check, PositionSensorNoiseCheck):
                self._widget.passed_position_sensor.setText(text)
            elif isinstance(check, MonotonicityCheck):
                self._widget.passed_monotonicity.setText(text)

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
