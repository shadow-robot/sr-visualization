#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import os
import rospkg
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QGridLayout,
    QRadioButton,
    QHBoxLayout,
    QGroupBox,
    QComboBox,
    QLabel,
    QFormLayout
)


class GenericTabOptions(QWidget):

    ICON_DIR = os.path.join(
            rospkg.RosPack().get_path('sr_visualization_icons'), 'icons')
    ICONS = {
        'GREEN': QIcon(os.path.join(ICON_DIR, 'green.png')),
        'RED': QIcon(os.path.join(ICON_DIR, 'red.png')),
        'BLUE': QIcon(os.path.join(ICON_DIR, 'blue.png')),
        'MAGENTA': QIcon(os.path.join(ICON_DIR, 'magenta.png')),
        'GRAY': QIcon(os.path.join(ICON_DIR, 'gray.png')),
        'CYAN': QIcon(os.path.join(ICON_DIR, 'cyan.png'))
    }

    def __init__(self, _tab_name, parent=None):
        super().__init__(parent=parent)
        self.init_ui()
        self.create_tab_options()
        self.setLayout(self.layout)

    def init_ui(self):
        self.layout = QGridLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_tab_options(self):
        groupbox = QGroupBox("Graph Options")
        self.layout.addWidget(groupbox)

        self.check_layout = QHBoxLayout()
        groupbox.setLayout(self.check_layout)

        self.create_variable_trace_buttons()
        self.create_common_buttons()

    def create_variable_trace_buttons(self):
        raise NotImplementedError("The function create_variable_trace_buttons must be implemented")

    def create_common_buttons(self):
        # pylint: disable=W0201
        self.all_button = QRadioButton("All")
        self.all_button.setObjectName("toggle_all")
        self.check_layout.addWidget(self.all_button)

        self.all_button.setChecked(True)

        self.show_seleted_button = QPushButton("Show Selected")
        self.show_seleted_button.setObjectName("show_seleted_button")
        self.check_layout.addWidget(self.show_seleted_button)

        self.reset_button = QPushButton("Reset")
        self.reset_button.setObjectName("reset_button")
        self.check_layout.addWidget(self.reset_button)


class JointStatesTabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        # pylint: disable=W0201
        self.position_button = QRadioButton("Position (rad)")
        self.position_button.setIcon(self.ICONS['RED'])
        self.check_layout.addWidget(self.position_button)

        self.effort_button = QRadioButton("Effort")
        self.effort_button.setIcon(self.ICONS['BLUE'])
        self.check_layout.addWidget(self.effort_button)

        self.velocity_button = QRadioButton("Velocity (rad/s)")
        self.velocity_button.setIcon(self.ICONS['GREEN'])
        self.check_layout.addWidget(self.velocity_button)


class ControlLoopsTabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        # pylint: disable=W0201
        self.setpoint_button = QRadioButton("Set Point (rad)")
        self.setpoint_button.setIcon(self.ICONS['RED'])
        self.check_layout.addWidget(self.setpoint_button)

        self.input_button = QRadioButton("Input (rad)")
        self.input_button.setIcon(self.ICONS['BLUE'])
        self.check_layout.addWidget(self.input_button)

        self.dinputdt_button = QRadioButton("dInput/dt (rad/s)")
        self.dinputdt_button.setIcon(self.ICONS['GREEN'])
        self.check_layout.addWidget(self.dinputdt_button)

        self.error_button = QRadioButton("Error")
        self.error_button.setIcon(self.ICONS['CYAN'])
        self.check_layout.addWidget(self.error_button)

        self.output_button = QRadioButton("Output")
        self.output_button.setIcon(self.ICONS['MAGENTA'])
        self.check_layout.addWidget(self.output_button)


class MotorStats1TabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        # pylint: disable=W0201
        self.strain_right_button = QRadioButton("Strain Gauge Right")
        self.strain_right_button.setIcon(self.ICONS['RED'])
        self.check_layout.addWidget(self.strain_right_button)

        self.strain_left_button = QRadioButton("Strain Gauge Left")
        self.strain_left_button.setIcon(self.ICONS['BLUE'])
        self.check_layout.addWidget(self.strain_left_button)

        self.pwm_button = QRadioButton("Measured PWM")
        self.pwm_button.setIcon(self.ICONS['GREEN'])
        self.check_layout.addWidget(self.pwm_button)

        self.current_button = QRadioButton("Measured Current (A)")
        self.current_button.setIcon(self.ICONS['CYAN'])
        self.check_layout.addWidget(self.current_button)

        self.voltage_button = QRadioButton("Measured Voltage (V)")
        self.voltage_button.setIcon(self.ICONS['MAGENTA'])
        self.check_layout.addWidget(self.voltage_button)


class MotorStats2TabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        # pylint: disable=W0201
        self.effort_button = QRadioButton("Measured Effort")
        self.effort_button.setIcon(self.ICONS['RED'])
        self.check_layout.addWidget(self.effort_button)

        self.temp_button = QRadioButton("Temperature (ºC)")
        self.temp_button.setIcon(self.ICONS['BLUE'])
        self.check_layout.addWidget(self.temp_button)

        self.unf_position_button = QRadioButton("Unfiltered Position")
        self.unf_position_button.setIcon(self.ICONS['GREEN'])
        self.check_layout.addWidget(self.unf_position_button)

        self.unf_force_button = QRadioButton("Unfiltered Force")
        self.unf_force_button.setIcon(self.ICONS['CYAN'])
        self.check_layout.addWidget(self.unf_force_button)

        self.last_effort_button = QRadioButton("Last Commanded Effort")
        self.last_effort_button.setIcon(self.ICONS['MAGENTA'])
        self.check_layout.addWidget(self.last_effort_button)

        self.encoder_pos_button = QRadioButton("Encoder Position (rad)")
        self.encoder_pos_button.setIcon(self.ICONS['GRAY'])
        self.check_layout.addWidget(self.encoder_pos_button)


class PalmExtrasAcellTabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        # pylint: disable=W0201
        self.accel_x_button = QRadioButton("Accel X")
        self.accel_x_button.setIcon(self.ICONS['RED'])
        self.check_layout.addWidget(self.accel_x_button)

        self.accel_y_button = QRadioButton("Accel Y")
        self.accel_y_button.setIcon(self.ICONS['BLUE'])
        self.check_layout.addWidget(self.accel_y_button)

        self.accel_z_button = QRadioButton("Accel Z")
        self.accel_z_button.setIcon(self.ICONS['GREEN'])
        self.check_layout.addWidget(self.accel_z_button)

    def create_common_buttons(self):
        # pylint: disable=W0201
        self.all_accel_button = QRadioButton("All")
        self.check_layout.addWidget(self.all_accel_button)

        self.all_accel_button.setChecked(True)


class PalmExtrasGyroTabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        # pylint: disable=W0201
        self.gyro_x_button = QRadioButton("Gyro X")
        self.gyro_x_button.setIcon(self.ICONS['CYAN'])
        self.check_layout.addWidget(self.gyro_x_button)

        self.gyro_y_button = QRadioButton("Gyro Y")
        self.gyro_y_button.setIcon(self.ICONS['MAGENTA'])
        self.check_layout.addWidget(self.gyro_y_button)

        self.gyro_z_button = QRadioButton("Gyro Z")
        self.gyro_z_button.setIcon(self.ICONS['GRAY'])
        self.check_layout.addWidget(self.gyro_z_button)

    def create_common_buttons(self):
        # pylint: disable=W0201
        self.all_gyro_button = QRadioButton("All")
        self.check_layout.addWidget(self.all_gyro_button)

        self.all_gyro_button.setChecked(True)


class PalmExtrasADCTabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        # pylint: disable=W0201
        self.adc0_button = QRadioButton("ADC0")
        self.adc0_button.setIcon(self.ICONS['RED'])
        self.check_layout.addWidget(self.adc0_button)

        self.adc1_button = QRadioButton("ADC1")
        self.adc1_button.setIcon(self.ICONS['BLUE'])
        self.check_layout.addWidget(self.adc1_button)

        self.adc2_button = QRadioButton("ADC2")
        self.adc2_button.setIcon(self.ICONS['GREEN'])
        self.check_layout.addWidget(self.adc2_button)

        self.adc3_button = QRadioButton("ADC3")
        self.adc3_button.setIcon(self.ICONS['CYAN'])
        self.check_layout.addWidget(self.adc3_button)

    def create_common_buttons(self):
        # pylint: disable=W0201
        self.all_adc_button = QRadioButton("All")
        self.check_layout.addWidget(self.all_adc_button)

        self.all_adc_button.setChecked(True)
