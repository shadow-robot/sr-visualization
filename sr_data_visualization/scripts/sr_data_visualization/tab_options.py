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

from PyQt5.QtWidgets import (
    QPushButton,
    QWidget,
    QGridLayout,
    QRadioButton,
    QHBoxLayout,
    QGroupBox,
)


class GenericTabOptions(QWidget):
    """
        Creates the options of filtering and selection for the tab
    """
    def __init__(self, tab_name, parent=None):
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
        pass

    def create_common_buttons(self):
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
    """
        Creates the options of filtering and selection for the tab
    """
    def __init__(self, tab_name, parent=None):
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        self.position_button = QRadioButton("Position")
        self.position_button.setObjectName("toggle_position")
        self.check_layout.addWidget(self.position_button)

        self.velocity_button = QRadioButton("Velocity")
        self.velocity_button.setObjectName("toggle_velocity")
        self.check_layout.addWidget(self.velocity_button)

        self.effort_button = QRadioButton("Effort")
        self.effort_button.setObjectName("toggle_effort")
        self.check_layout.addWidget(self.effort_button)


class ControlLoopsTabOptions(GenericTabOptions):
    """
        Creates the options of filtering and selection for the tab
    """
    def __init__(self, tab_name, parent=None):
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        self.setpoint_button = QRadioButton("Set Point")
        self.setpoint_button.setObjectName("toggle_setpoint")
        self.check_layout.addWidget(self.setpoint_button)

        self.input_button = QRadioButton("Input")
        self.input_button.setObjectName("toggle_input")
        self.check_layout.addWidget(self.input_button)

        self.dinputdt_button = QRadioButton("dInput/dt")
        self.dinputdt_button.setObjectName("toggle_dinputdt")
        self.check_layout.addWidget(self.dinputdt_button)

        self.error_button = QRadioButton("Error")
        self.error_button.setObjectName("toggle_error")
        self.check_layout.addWidget(self.error_button)

        self.output_button = QRadioButton("Output")
        self.output_button.setObjectName("toggle_output")
        self.check_layout.addWidget(self.output_button)


class MotorStats1TabOptions(GenericTabOptions):
    """
        Creates the options of filtering and selection for the tab
    """
    def __init__(self, tab_name, parent=None):
        super().__init__(tab_name, parent)

    def create_variable_trace_buttons(self):
        self.strain_right_button = QRadioButton("Strain Gauge Right")
        # self.strain_right_button.setObjectName("toggle_strain_right")
        self.check_layout.addWidget(self.strain_right_button)

        self.strain_left_button = QRadioButton("Strain Gauge Left")
        # self.strain_left_button.setObjectName("toggle_input")
        self.check_layout.addWidget(self.strain_left_button)

        self.pwm_button = QRadioButton("Measured PWM")
        # self.pwm_button.setObjectName("toggle_dinputdt")
        self.check_layout.addWidget(self.pwm_button)

        self.current_button = QRadioButton("Measured Current")
        # self.current_button.setObjectName("toggle_error")
        self.check_layout.addWidget(self.current_button)

        self.voltage_button = QRadioButton("Measured Voltage")
        # self.voltage_button.setObjectName("toggle_output")
        self.check_layout.addWidget(self.voltage_button)
