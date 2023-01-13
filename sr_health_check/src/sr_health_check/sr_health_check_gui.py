# Copyright 2011, 2022-2023 Shadow Robot Company Ltd.
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
import threading
import queue
from datetime import datetime
from collections import OrderedDict
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget, QApplication, QTreeWidgetItem
from python_qt_binding.QtGui import QColor

from qt_gui.plugin import Plugin

from sr_hand_health_report.monotonicity_check import MonotonicityCheck
from sr_hand_health_report.position_sensor_noise_check import PositionSensorNoiseCheck
from sr_hand_health_report.motor_check import MotorCheck
from sr_hand_health_report.tactile_check import TactileCheck
from sr_hand_health_report.backlash_check import BacklashCheck
from sr_hand_health_report.overrun_check import OverrunCheck

import yaml
from rosgraph_msgs.msg import Log


class SrHealthCheck(Plugin):

    """
    A GUI plugin for health check execution.

    """

    FAIL_COLOR = QColor.fromRgb(255, 100, 100)

    def __init__(self, context):
        super().__init__(context)

        self.setObjectName('SrGuiHealthCheck')
        self._results_file = f"{rospkg.RosPack().get_path('sr_health_check')}/src/sr_health_check/results.yaml"
        self._entry_name = None

        self._timer = QTimer()
        self._timer.timeout.connect(self.timerEvent)
        self._results = OrderedDict()

        self._fingers = ('FF', 'MF', 'RF', 'LF', "TH", "WR")
        self._side = "right"  # to be parametrized later
        self._check_names = ['motor', 'position_sensor_noise', 'monotonicity', 'tactile', 'backlash', 'overrun']

        self._checks_to_execute = {}
        self._check_queue = queue.Queue()

        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_health_check'), 'uis', 'SrHealthCheck.ui')
        loadUi(ui_file, self._widget)

        if context:
            context.add_widget(self._widget)

        self._current_data = self.get_data_from_results_file()
        self._selected_checks = dict.fromkeys(self._check_names, False)

        self.initialize_checks()
        self.setup_connections()
        self._timer.start(100)

        self.display_data()

        self._rqt_node_name = rospy.get_name()
        rospy.Subscriber("/rosout", Log, self.status_subscriber)

    def status_subscriber(self, msg):
        if msg.name == self._rqt_node_name:
            status = msg.msg
            self._widget.label_status.setText(f"Status:{status}")

    def timerEvent(self):  # pylint: disable=C0103
        checks_are_running = False
        for check_name in self._check_names:
            if self._checks_to_execute[check_name]['thread'].is_alive():
                checks_are_running = True
        selected_checks = any(check for check in self._check_names if self._selected_checks[check])
        self._widget.button_start_selected.setEnabled(not checks_are_running and selected_checks)
        self._widget.button_start_all.setEnabled(not checks_are_running)

    def initialize_checks(self):

        for check_name in self._check_names:
            self._checks_to_execute[check_name] = {'check': 0, 'thread': 0}

        self._checks_to_execute['motor']['check'] = MotorCheck(self._side, self._fingers)
        self._checks_to_execute['position_sensor_noise']['check'] = PositionSensorNoiseCheck(self._side, self._fingers)
        self._checks_to_execute['monotonicity']['check'] = MonotonicityCheck(self._side, self._fingers)
        self._checks_to_execute['tactile']['check'] = TactileCheck(self._side)
        self._checks_to_execute['backlash']['check'] = BacklashCheck(self._side, self._fingers)
        self._checks_to_execute['overrun']['check'] = OverrunCheck(self._side, self._fingers)

        for name in self._check_names:
            self._checks_to_execute[name]['thread'] = \
                threading.Thread(target=self._checks_to_execute[name]['check'].run_check)

    def setup_connections(self):
        #  Creates connections with buttons
        self._widget.button_start_selected.clicked.connect(self.button_start_selected_clicked)
        self._widget.button_start_all.clicked.connect(self.button_start_all_clicked)

        #  Creates connections with checkboxes
        self._widget.checkbox_motor.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_position_sensor.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_monotonicity.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_tactile.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_backlash.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_overrun.clicked.connect(self.checkbox_selected)

        #  Creates connections with date combobox
        self._widget.combobox_date.activated.connect(self.combobox_selected)
        #  Creates connections with tabs
        self._widget.tab_widget.currentChanged.connect(self.tab_changed)

        threading.Thread(target=self.check_execution, daemon=True).start()

    def button_start_selected_clicked(self):
        self._entry_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._entry_name] = {}
        for check_name in self._check_names:
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
            if self._selected_checks[check_name]:
                self._checks_to_execute[check_name]['thread'] = \
                    threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
                self._check_queue.put(self._checks_to_execute[check_name])

    def button_start_all_clicked(self):
        self._entry_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._entry_name] = {}
        for check_name in self._check_names:
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
            self._checks_to_execute[check_name]['thread'] = \
                threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
            self._check_queue.put(self._checks_to_execute[check_name])

    def checkbox_selected(self):
        self._selected_checks['motor'] = self._widget.checkbox_motor.isChecked()
        self._selected_checks['position_sensor_noise'] = self._widget.checkbox_position_sensor.isChecked()
        self._selected_checks['monotonicity'] = self._widget.checkbox_monotonicity.isChecked()
        self._selected_checks['tactile'] = self._widget.checkbox_tactile.isChecked()
        self._selected_checks['backlash'] = self._widget.checkbox_backlash.isChecked()
        self._selected_checks['overrun'] = self._widget.checkbox_overrun.isChecked()

    def check_execution(self):
        while not rospy.is_shutdown():
            check = self._check_queue.get()
            check['thread'].start()
            self.update_passed_label(check['check'], "Executing")
            check['thread'].join()

            self._check_queue.task_done()

            self._results[self._entry_name].update(check['check'].get_result())
            self.update_passed_label(check['check'])

            if self._check_queue.empty():
                self.save_results_to_result_file(self._results)

    def get_data_from_results_file(self):
        output = {}
        try:
            with open(self._results_file, 'r', encoding="ASCII") as yaml_file:
                output = yaml.safe_load(yaml_file) or {}
        except FileNotFoundError:
            rospy.logwarn("Result file does not exists. Returning empty dictionary")
        return output

    def save_results_to_result_file(self, results):
        if results:
            self._current_data.update(results)
            with open(self._results_file, 'w', encoding="ASCII") as yaml_file:
                yaml.safe_dump(self._current_data, stream=yaml_file, default_flow_style=False)

    def update_passed_label(self, check, text=None):
        if isinstance(check, MotorCheck):
            self._widget.passed_motor.setText(text or str(self._checks_to_execute['motor']
                                              ['check'].has_passed()))
        elif isinstance(check, PositionSensorNoiseCheck):
            self._widget.passed_position_sensor.setText(text or str(self._checks_to_execute['position_sensor_noise']
                                                        ['check'].has_passed()))
        elif isinstance(check, MonotonicityCheck):
            self._widget.passed_monotonicity.setText(text or str(self._checks_to_execute['monotonicity']
                                                     ['check'].has_passed()))
        elif isinstance(check, TactileCheck):
            self._widget.passed_tactile.setText(text or str(self._checks_to_execute['tactile']
                                                ['check'].has_passed()))
        elif isinstance(check, BacklashCheck):
            self._widget.passed_backlash.setText(text or str(self._checks_to_execute['backlash']
                                                 ['check'].has_passed()))
        elif isinstance(check, OverrunCheck):
            self._widget.passed_overrun.setText(text or str(self._checks_to_execute['overrun']
                                                ['check'].has_passed()))

    def tab_changed(self, _):
        if self._widget.tab_widget.currentWidget().accessibleName() == "tab_view":
            self.combobox_selected()

    def combobox_selected(self):
        self.display_data()

    def display_data(self):
        if not bool(self._current_data):
            return

        for data_entry in list(self._current_data.keys()):
            if self._widget.combobox_date.findText(data_entry) == -1:  # findText returns the index of of the item or -1
                self._widget.combobox_date.insertItem(0, data_entry)

        self._widget.treeWidget.clear()
        date = self._widget.combobox_date.currentText()
        self.update_tree(self._current_data[date], None)

    def update_tree(self, data, parent):
        if isinstance(data, dict):
            for key in data.keys():
                item = QTreeWidgetItem(parent, [str(key)])
                if self._widget.treeWidget.indexOfTopLevelItem(item) == -1:
                    self._widget.treeWidget.addTopLevelItem(item)
                item.addChild(self.update_tree(data[key], item))
                if item.background(0) == SrHealthCheck.FAIL_COLOR and item.parent():
                    item.parent().setBackground(0, SrHealthCheck.FAIL_COLOR)
        else:
            item = QTreeWidgetItem(parent, [str(data)])
            check_name = self.get_top_parent_name(item)
            if not self._checks_to_execute[check_name]['check'].has_single_passed(parent.text(0), data):
                item.setBackground(0, SrHealthCheck.FAIL_COLOR)
                item.parent().setBackground(0, SrHealthCheck.FAIL_COLOR)
            return item
        return

    @staticmethod
    def get_top_parent_name(self, item):
        name = None
        while item:
            name = item.text(0)
            item = item.parent()
        return name

    def get_widget(self):
        return self._widget


if __name__ == "__main__":
    rospy.init_node("sr_health_check_gui")
    app = QApplication(sys.argv)
    ctrl = SrHealthCheck(None)
    ctrl.get_widget().show()  # pylint: disable=W0212
    app.exec_()
