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
from python_qt_binding.QtGui import QColor, QFont


from qt_gui.plugin import Plugin

from sr_hand_health_report.monotonicity_check import MonotonicityCheck
from sr_hand_health_report.position_sensor_noise_check import PositionSensorNoiseCheck
from sr_hand_health_report.motor_check import MotorCheck
from sr_hand_health_report.tactile_check import TactileCheck
from sr_hand_health_report.backlash_check import BacklashCheck
from sr_hand_health_report.overrun_check import OverrunCheck

import yaml
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState


class SrHealthCheck(Plugin):

    '''
    A GUI plugin for health check execution.

    '''

    FAIL_COLOR = QColor.fromRgb(255, 100, 100)
    _SIDE_PREFIXES = ('rh', 'lh')

    def __init__(self, context):
        '''
            Initializes the Health Metrics plugin.
        '''
        super().__init__(context)

        self.setObjectName('SrGuiHealthCheck')
        self._results_file = f"{rospkg.RosPack().get_path('sr_health_check')}/src/sr_health_check/results.yaml"
        self._entry_name = None

        self._timer = QTimer()
        self._timer.timeout.connect(self.timerEvent)
        self._results = OrderedDict()

        self._fingers = ('FF', 'MF', 'RF', 'LF', "TH", "WR")
        self._side = "right"  # to be parametrized later
        self._detected_prefixes = []
        self._check_names = ['motor', 'position_sensor_noise', 'monotonicity', 'tactile', 'backlash', 'overrun']

        self._checks_to_execute = {}
        self._check_queue = queue.Queue()
        self._checks_running = False

        self._widget = QWidget()
        self._bold_font = QFont()
        self._bold_font.setBold(True)

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_health_check'), 'uis', 'SrHealthCheck.ui')
        loadUi(ui_file, self._widget)

        if context:
            context.add_widget(self._widget)
        
        self._current_data = self.get_data_from_results_file()
        self._selected_checks = dict.fromkeys(self._check_names, False)

        self.initialize_checks()
        self._setup_connections()
        self._timer.start(100)

        self.display_data()
        self.set_side_selection_accesibility()

        self._rqt_node_name = rospy.get_name()
        self._log_subcriber = rospy.Subscriber("/rosout", Log, self._status_subscriber)
        self._log_publisher = rospy.Publisher("/rosout", Log, queue_size=1)

    def set_side_selection_accesibility(self):
        '''
            Sets the visibility of side selection radio buttons.
        '''
        checkable_side_selection_radio_buttons = []
        joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=2)
        joint_names = [joint_name.split("_")[0] for joint_name in joint_states_msg.name]

        for side_prefix in SrHealthCheck._SIDE_PREFIXES:
            if side_prefix in joint_names:
                checkable_side_selection_radio_buttons.append(side_prefix)
                self._detected_prefixes.append(side_prefix)

        if 'lh' in checkable_side_selection_radio_buttons:
            self._widget.side_left_radio_button.setChecked(True)
            self._widget.side_left_radio_button.click()
        if 'rh' in checkable_side_selection_radio_buttons:
            self._widget.side_right_radio_button.setChecked(True)
            self._widget.side_right_radio_button.click()

    def _status_subscriber(self, msg):
        '''
            Callback method for /rosout subscriber.
            @param msg: rosgraph_msgs.msg.Log type message
        '''
        if msg.name == self._rqt_node_name:
            status = msg.msg
            self._widget.label_status.setText(f"Status:{status}")

    def timerEvent(self):  # pylint: disable=C0103
        '''
            Timer event to enable/disable the start checks butons.
        '''
        selected_checks = any(check for check in self._check_names if self._selected_checks[check])
        self._widget.button_start_selected.setEnabled(not self._checks_running and selected_checks)
        self._widget.button_start_all.setEnabled(not self._checks_running)
        self._widget.button_stop.setEnabled(self._checks_running)
        self._widget.side_right_radio_button.setEnabled(not self._checks_running and "rh" in self._detected_prefixes)
        self._widget.side_left_radio_button.setEnabled(not self._checks_running  and "lh" in self._detected_prefixes)

    def are_checks_running(self):
        '''
            Gets if any checks are running (any checks left in the queue)
            @return bool
        '''
        return self._checks_running 

    def initialize_checks(self):
        '''
            Initializes the dictionary containing references to check classes and threads.
        '''
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

    '''
        Sets up the connections between buttons and corresponding actions.
    '''
    def _setup_connections(self):
        #  Creates connections with start/stop buttons
        self._widget.button_start_selected.clicked.connect(self._button_start_selected_clicked)
        self._widget.button_start_all.clicked.connect(self._button_start_all_clicked)
        self._widget.button_stop.clicked.connect(self._button_stop_clicked)

        #  Creates connections with check selection checkboxes
        self._widget.checkbox_motor.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_position_sensor.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_monotonicity.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_tactile.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_backlash.clicked.connect(self.checkbox_selected)
        self._widget.checkbox_overrun.clicked.connect(self.checkbox_selected)

        #  Creates connections side selection radio buttons
        self._widget.side_right_radio_button.clicked.connect(self.side_selected)
        self._widget.side_left_radio_button.clicked.connect(self.side_selected)

        #  Creates connections with date combobox
        self._widget.combobox_date.activated.connect(self.combobox_selected)

        threading.Thread(target=self.check_execution, daemon=True).start()

    def _button_start_selected_clicked(self):
        '''
            Performs actions upon 'Start' button click.
        '''
        self._entry_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._entry_name] = {}
        self._checks_running = True
        for check_name in self._check_names:
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
            self._checks_to_execute[check_name]['check'].set_runnable()
            if self._selected_checks[check_name]:
                self._checks_to_execute[check_name]['thread'] = \
                    threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
                self._check_queue.put(self._checks_to_execute[check_name])

    def _button_start_all_clicked(self):
        '''
            Performs actions upon 'Start all' button click.
        '''
        self._entry_name = datetime.now().strftime("%d-%m-%Y-%H-%M-%S")
        self._results[self._entry_name] = {}
        self._checks_running = True
        for check_name in self._check_names:
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
            self._checks_to_execute[check_name]['check'].set_runnable()
            self._checks_to_execute[check_name]['thread'] = \
                threading.Thread(target=self._checks_to_execute[check_name]['check'].run_check)
            self._check_queue.put(self._checks_to_execute[check_name])

    def _button_stop_clicked(self):
        '''
            Performs actions upon 'Stop' button click.
        '''
        for check_name in self._check_names:
            self._checks_to_execute[check_name]['check'].stop_test()
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
        self.send_status_message("Checks stopped. Results not beinng saved!")
        '''
        while not self._check_queue.empty():
            rospy.sleep(0.1)
            try:
                self._check_queue.pop(0)
            except Exception:
                break

        self._checks_running = False
        for check_name in self._check_names:
            self.update_passed_label(self._checks_to_execute[check_name]['check'], "-")
        self.send_status_message("Check execution stopped!")
        '''

    def checkbox_selected(self):
        '''
            Gets values from check checkboxes.
        '''
        self._selected_checks['motor'] = self._widget.checkbox_motor.isChecked()
        self._selected_checks['position_sensor_noise'] = self._widget.checkbox_position_sensor.isChecked()
        self._selected_checks['monotonicity'] = self._widget.checkbox_monotonicity.isChecked()
        self._selected_checks['tactile'] = self._widget.checkbox_tactile.isChecked()
        self._selected_checks['backlash'] = self._widget.checkbox_backlash.isChecked()
        self._selected_checks['overrun'] = self._widget.checkbox_overrun.isChecked()

    def side_selected(self):
        '''
            Initializes checks based on selected side.
        '''
        if self._widget.side_right_radio_button.isChecked():
            self._side = "right" 
        elif self._widget.side_left_radio_button.isChecked():
            self._side = "left" 
        self.initialize_checks()

    def check_execution(self):
        '''
            Executes queued checks and saves result to a file.
        '''
        while not rospy.is_shutdown():
            check = self._check_queue.get()
            result = None
            self._checks_running = True

            if not check['check'].is_stopped():
                check['thread'].start()            
                self.update_passed_label(check['check'], "Executing")
                check['thread'].join()
                result = check['check'].get_result()

                if list(result.values())[0]:
                    self.update_passed_label(check['check'])
                    self._results[self._entry_name].update(result)
                else:
                    self.update_passed_label(check['check'], "-")
                    del self._results[self._entry_name]

            self._check_queue.task_done()
            
            if self._check_queue.empty():
                if result and check['check'].is_stopped() and bool(list(result.values())[0]):
                    self.save_results_to_result_file(self._results)
                    self.send_status_message("Checks completed. Results saved!")
                    self.display_data()
                self._checks_running = False

            '''
            if result:
                self._results[self._entry_name].update(result)
                print(result)

                self.update_passed_label(check['check'])

                if self.are_checks_running():
                    if not check['check'].is_stopped():
                        self.save_results_to_result_file(self._results)
                    self.display_data()
                    self.send_status_message("Checks completed!")
                    self._checks_running = False
            '''

    def send_status_message(self, message):
        '''
            Sends a status message at rospy.INFO level
            @param message: rosgraph_msgs.msg.Log type message
        '''
        msg = Log()
        msg.name = self._rqt_node_name
        msg.level = rospy.INFO
        msg.msg = message
        self._log_publisher.publish(msg)

    def get_data_from_results_file(self):
        '''
            Gets the results file content.
            @return dict: The contents of the result file
        '''
        output = {}
        try:
            with open(self._results_file, 'r', encoding="ASCII") as yaml_file:
                output = yaml.safe_load(yaml_file) or {}
        except FileNotFoundError:
            pass
            rospy.logwarn("Result file does not exists. Creating a new log file!")
        return output

    def save_results_to_result_file(self, results):
        '''
            Saves the results to the result file.
            @param results: results to be saved
        '''
        if results:
            self._current_data.update(results)
            with open(self._results_file, 'w', encoding="ASCII") as yaml_file:
                yaml.safe_dump(self._current_data, stream=yaml_file, default_flow_style=False)

    def update_passed_label(self, check, text=None):
        '''
            Updates the passed label in the GUI with feedback on completion status.
            @param check: Check object
            @param text: String stating the state of the check.
        '''
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

    def combobox_selected(self):
        '''
            Performs action upon date combobox selection from View tab
        '''
        self.display_data()

    def display_data(self):
        '''
            Updates the results tree to be presented.
        '''
        if not bool(self._current_data):
            return

        for data_entry in list(self._current_data.keys()):
            if self._widget.combobox_date.findText(data_entry) == -1:  # findText returns the index of of the item or -1
                self._widget.combobox_date.insertItem(0, data_entry)

        self._widget.treeWidget.clear()
        date = self._widget.combobox_date.currentText()
        if date != '':
            self.update_tree(self._current_data[date], None)

    def update_tree(self, data, parent):  # pylint: disable=R1710
        '''
            Creates the results tree to be presented.
            @param data: dictionary containing the data
            @param parent: parent for lower level elements
        '''
        if isinstance(data, dict):
            for key in data.keys():
                item = QTreeWidgetItem(parent, [str(key)])
                if self._widget.treeWidget.indexOfTopLevelItem(item) == -1:
                    self._widget.treeWidget.addTopLevelItem(item)
                item.addChild(self.update_tree(data[key], item))
                if item.foreground(0) == SrHealthCheck.FAIL_COLOR and item.parent():
                    item.parent().setForeground(0, SrHealthCheck.FAIL_COLOR)
                    item.parent().setFont(0, self._bold_font)
        else:
            item = QTreeWidgetItem(parent, [str(data)])
            check_name = SrHealthCheck.get_top_parent_name(item)
            if not self._checks_to_execute[check_name]['check'].has_single_passed(parent.text(0), data):
                item.setForeground(0, SrHealthCheck.FAIL_COLOR)
                item.parent().setForeground(0, SrHealthCheck.FAIL_COLOR)
                item.setFont(0, self._bold_font)
                item.parent().setFont(0, self._bold_font)
            return item
        return


    @staticmethod
    def get_top_parent_name(item):
        '''
            Gets the top parent from child of a nested QTreeWidget
            @param item: Child of the QTreeWidget
            @return string: Name of the top level parent of the item.
        '''
        name = None
        while item:
            name = item.text(0)
            item = item.parent()
        return name

    def get_widget(self):
        '''
            Returns the widget
            @return QWidget
        '''
        return self._widget

    def shutdown_plugin(self):
        '''
            Shutdown hook called whenever the plugin exits.
        '''
        self._log_subcriber.unregister()


if __name__ == "__main__":
    rospy.init_node("sr_health_check_gui")
    app = QApplication(sys.argv)
    ctrl = SrHealthCheck(None)
    ctrl.get_widget().show()  # pylint: disable=W0212
    app.exec_()
