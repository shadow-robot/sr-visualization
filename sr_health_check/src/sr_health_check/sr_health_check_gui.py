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
from QtCore import Qt
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

class SrHealthCheck(Plugin):

    """
    A GUI plugin for bootloading the motors on the shadow etherCAT hand.
    """

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('SrGuiHealthCheck')
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_health_check'), 'uis', 'SrHealthCheck.ui')
        loadUi(ui_file, self._widget)
        self._widget.setLayout(QVBoxLayout())

        if context:
            context.add_widget(self._widget)

        self.setup_connections()

    def setup_connections(self):
        #self._widget.checkbox_motor.clicked.connect(self.checkbox_motor_clicked)
        #self._widget.checkbox_position_sensor.clicked.connect(self.checkbox_position_sensor_clicked)
        self._widget.button_start_selected.clicked.connect(self.button_start_selected_clicked)
        #self._widget.button_start_all.clicked.connect(self.button_start_all_clicked)
        #self._widget.button_stop.clicked.connect(self.button_stop_clicked)

    def button_start_selected_clicked(self):
        is_checked = {}
        is_checked['motor_check'] = self._widget.checkbox_motor.isChecked()
        is_checked['position_sensor_noise'] = self._widget.checkbox_position_sensor.isChecked()
        '''
        to be extended with more checks
        '''
        rospy.logwarn(is_checked)
        for key, value in is_checked.items():
            if value:
                self.execute_test(key)

    def execute_test(self, test):
        side = 'right'
        fingers = ['FF']
        if 'motor_check' == test:
            result = MonotonicityCheck(side, fingers).run_check()
        rospy.logwarn(result)
        return result

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
