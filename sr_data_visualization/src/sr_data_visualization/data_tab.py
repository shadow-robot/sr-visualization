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
from python_qt_binding.QtWidgets import (
    QWidget,
    QGridLayout,
    QVBoxLayout,
)
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64MultiArray
from sr_data_visualization.joint_graph_widget import JointGraph
from sr_data_visualization.data_plot import (
    JointStatesDataPlot,
    ControlLoopsDataPlot,
    MotorStats1DataPlot,
    MotorStats2DataPlot,
    PalmExtrasAcellDataPlot,
    PalmExtrasGyroDataPlot,
    PalmExtrasADCDataPlot
)
from sr_data_visualization.tab_options import (
    JointStatesTabOptions,
    ControlLoopsTabOptions,
    MotorStats1TabOptions,
    MotorStats2TabOptions,
    PalmExtrasAcellTabOptions,
    PalmExtrasGyroTabOptions,
    PalmExtrasADCTabOptions
)


class GenericDataTab(QWidget):
    MAX_NO_COLUMNS = 4

    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        QWidget.__init__(self, parent=parent)

        self.tab_name = tab_name
        self.hand_joints = hand_joints
        self.joint_prefix = joint_prefix
        self.init_ui()
        self.create_full_tab()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_full_tab(self):
        self.create_tab_options()

        self.graphs_layout = QGridLayout()
        self.create_all_graphs()

        self.optional_button_connections()
        self.generic_button_connections()

    def create_tab_options(self):
        raise NotImplementedError("The function create_tab_options must be implemented")

    def create_all_graphs(self):
        raise NotImplementedError("The function create_all_graphs must be implemented")

    def optional_button_connections(self):
        raise NotImplementedError("The function optional_button_connections must be implemented")

    def generic_button_connections(self):
        self.tab_options.all_button.toggled.connect(lambda: self.radio_button_selected("All"))
        self.tab_options.show_seleted_button.clicked.connect(lambda: self.check_button_selected("Selection"))
        self.tab_options.reset_button.clicked.connect(lambda: self.check_button_selected("All"))

    def radio_button_selected(self, radio_button):
        for child in self.findChildren(JointGraph):
            child.joint_plot.show_trace(radio_button)

    def check_button_selected(self, selection_type):
        index_to_display = 0
        for child in self.findChildren(JointGraph):
            if selection_type == "Selection":
                if not child.joint_check_box.isChecked():
                    child.hide()
                else:
                    self.graphs_layout.addWidget(child,
                                                 index_to_display // self.MAX_NO_COLUMNS,
                                                 index_to_display % self.MAX_NO_COLUMNS)
                    index_to_display += 1
            elif selection_type == "All":
                if child.joint_check_box.isChecked():
                    child.joint_check_box.setCheckState(False)
                    self.graphs_layout.addWidget(child, child.initial_row, child.initial_column)
                else:
                    child.show()

    def change_side(self, side):
        for child in self.findChildren(JointGraph):
            child.change_side(side)


class JointStatesDataTab(GenericDataTab):
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        # pylint: disable=R1702
        super().__init__(tab_name, hand_joints, joint_prefix, parent)
        self.tab_options = None

    def create_tab_options(self):
        self.tab_options = JointStatesTabOptions(self.tab_name)
        self.layout.addWidget(self.tab_options)

    def create_all_graphs(self):
        joints = {
            0: [],
            1: [],
            2: [],
            3: [],
            4: [],
            5: []
        }

        # Removing _ from self.joint_prefix (e.g. _ from rh_)
        for joint in self.hand_joints[self.joint_prefix[:-1]]:
            if "_THJ" in joint:
                joints[0].append(joint)
            elif "_FFJ" in joint:
                joints[1].append(joint)
            elif "_MFJ" in joint:
                joints[2].append(joint)
            elif "_RFJ" in joint:
                joints[3].append(joint)
            elif "_LFJ" in joint:
                joints[4].append(joint)
            elif "_WRJ" in joint:
                joints[5].append(joint)

        for column, joint_names in joints.items():
            row = 0
            for joint in joint_names:
                data_plot = JointStatesDataPlot(joint, 'joint_states', JointState)
                graph = JointGraph(joint, data_plot, row, column)
                self.graphs_layout.addWidget(graph, row, column)
                row += 1

        self.layout.addLayout(self.graphs_layout)

    def optional_button_connections(self):
        self.tab_options.position_button.toggled.connect(lambda: self.radio_button_selected("Position"))
        self.tab_options.velocity_button.toggled.connect(lambda: self.radio_button_selected("Velocity"))
        self.tab_options.effort_button.toggled.connect(lambda: self.radio_button_selected("Effort"))


class MotorGroupsDataTab(GenericDataTab):  # pylint: disable=W0223
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, hand_joints, joint_prefix, parent)

    def create_all_graphs(self):
        joints = {
            0: [],
            1: [],
            2: [],
            3: [],
            4: [],
            5: []
        }

        for joint in self.hand_joints[self.joint_prefix[:-1]]:
            if "_THJ" in joint:
                joints[0].append(joint)
            elif "_FFJ" in joint:
                if "J1" in joint:
                    joints[1].append(joint[:-1] + "0")
                elif "J2" not in joint:
                    joints[1].append(joint)
            elif "_MFJ" in joint:
                if "J1" in joint:
                    joints[2].append(joint[:-1] + "0")
                elif "J2" not in joint:
                    joints[2].append(joint)
            elif "_RFJ" in joint:
                if "J1" in joint:
                    joints[3].append(joint[:-1] + "0")
                elif "J2" not in joint:
                    joints[3].append(joint)
            elif "_LFJ" in joint:
                if "J1" in joint:
                    joints[4].append(joint[:-1] + "0")
                elif "J2" not in joint:
                    joints[4].append(joint)
            elif "_WRJ" in joint:
                joints[5].append(joint)

        for column, joint_names in joints.items():
            row = 0
            if joint_names is not None:
                for joint in joint_names:
                    control_topic_name = '/sh_' + joint.lower() + '_position_controller/state'
                    motor_topic_name = '/diagnostics_agg'
                    if self.tab_name == "Control Loops":
                        data_plot = ControlLoopsDataPlot(joint, control_topic_name,
                                                         JointControllerState)
                    elif self.tab_name == "Motor Stats 1":
                        data_plot = MotorStats1DataPlot(joint, motor_topic_name,
                                                        DiagnosticArray)
                    elif self.tab_name == "Motor Stats 2":
                        data_plot = MotorStats2DataPlot(joint, motor_topic_name,
                                                        DiagnosticArray)
                    graph = JointGraph(joint, data_plot, row, column)
                    self.graphs_layout.addWidget(graph, row, column)
                    row += 1

        self.layout.addLayout(self.graphs_layout)


class ControlLoopsDataTab(MotorGroupsDataTab):
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, hand_joints, joint_prefix, parent)
        self.tab_options = None

    def create_tab_options(self):
        self.tab_options = ControlLoopsTabOptions(self.tab_name)
        self.layout.addWidget(self.tab_options)

    def optional_button_connections(self):
        self.tab_options.setpoint_button.toggled.connect(lambda: self.radio_button_selected("Set Point"))
        self.tab_options.input_button.toggled.connect(lambda: self.radio_button_selected("Input"))
        self.tab_options.dinputdt_button.toggled.connect(lambda: self.radio_button_selected("dInput/dt"))
        self.tab_options.error_button.toggled.connect(lambda: self.radio_button_selected("Error"))
        self.tab_options.output_button.toggled.connect(lambda: self.radio_button_selected("Output"))


class MotorStats1DataTab(MotorGroupsDataTab):
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, hand_joints, joint_prefix, parent)
        self.tab_options = None

    def create_tab_options(self):
        self.tab_options = MotorStats1TabOptions(self.tab_name)
        self.layout.addWidget(self.tab_options)

    def optional_button_connections(self):
        self.tab_options.strain_right_button.toggled.connect(lambda: self.radio_button_selected("Strain Gauge Right"))
        self.tab_options.strain_left_button.toggled.connect(lambda: self.radio_button_selected("Strain Gauge Left"))
        self.tab_options.pwm_button.toggled.connect(lambda: self.radio_button_selected("Measured PWM"))
        self.tab_options.current_button.toggled.connect(lambda: self.radio_button_selected("Measured Current"))
        self.tab_options.voltage_button.toggled.connect(lambda: self.radio_button_selected("Measured Voltage"))


class MotorStats2DataTab(MotorGroupsDataTab):
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, hand_joints, joint_prefix, parent)
        self.tab_options = None

    def create_tab_options(self):
        self.tab_options = MotorStats2TabOptions(self.tab_name)
        self.layout.addWidget(self.tab_options)

    def optional_button_connections(self):
        self.tab_options.effort_button.toggled.connect(lambda: self.radio_button_selected("Measured Effort"))
        self.tab_options.temp_button.toggled.connect(lambda: self.radio_button_selected("Temperature"))
        self.tab_options.unf_position_button.toggled.connect(lambda: self.radio_button_selected("Unfiltered position"))
        self.tab_options.unf_force_button.toggled.connect(lambda: self.radio_button_selected("Unfiltered force"))
        self.tab_options.last_effort_button.toggled.connect(lambda: self.radio_button_selected("Last Commanded Effort"))
        self.tab_options.encoder_pos_button.toggled.connect(lambda: self.radio_button_selected("Encoder Position"))


class PalmExtrasDataTab(GenericDataTab):  # pylint: disable=W0223
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        # pylint: disable=W0235
        super().__init__(tab_name, hand_joints, joint_prefix, parent)

    def create_full_tab(self):
        topic_name = '/' + self.joint_prefix[:-1] + '/palm_extras'
        topic_type = Float64MultiArray

        self.accel_tab_options = PalmExtrasAcellTabOptions(self.tab_name)
        self.layout.addWidget(self.accel_tab_options)

        self.accel_data_plot = PalmExtrasAcellDataPlot("Acceleration", topic_name, topic_type)
        accel_graph = JointGraph("Acceleration", self.accel_data_plot, 0, 0, check_box=False)
        self.layout.addWidget(accel_graph)

        self.layout.addStretch(1)

        self.gyro_tab_options = PalmExtrasGyroTabOptions(self.tab_name)
        self.layout.addWidget(self.gyro_tab_options)

        self.gyro_data_plot = PalmExtrasGyroDataPlot("Gyrometer", topic_name, topic_type)
        gyro_graph = JointGraph("Gyrometer", self.gyro_data_plot, 1, 0, check_box=False)
        self.layout.addWidget(gyro_graph)

        self.adc_tab_options = PalmExtrasADCTabOptions(self.tab_name)
        self.layout.addWidget(self.adc_tab_options)

        self.adc_data_plot = PalmExtrasADCDataPlot("ADC", topic_name, topic_type)
        adc_graph = JointGraph("ADC", self.adc_data_plot, 1, 0, check_box=False)
        self.layout.addWidget(adc_graph)

        self.optional_button_connections()
        self.generic_button_connections()

    def optional_button_connections(self):
        self.accel_tab_options.accel_x_button.toggled.connect(lambda: self.radio_button_selected("Accel X", "accel"))
        self.accel_tab_options.accel_y_button.toggled.connect(lambda: self.radio_button_selected("Accel Y", "accel"))
        self.accel_tab_options.accel_z_button.toggled.connect(lambda: self.radio_button_selected("Accel Z", "accel"))

        self.gyro_tab_options.gyro_x_button.toggled.connect(lambda: self.radio_button_selected("Gyro X", "gyro"))
        self.gyro_tab_options.gyro_y_button.toggled.connect(lambda: self.radio_button_selected("Gyro Y", "gyro"))
        self.gyro_tab_options.gyro_z_button.toggled.connect(lambda: self.radio_button_selected("Gyro Z", "gyro"))

        self.adc_tab_options.adc0_button.toggled.connect(lambda: self.radio_button_selected("ADC0", "adc"))
        self.adc_tab_options.adc1_button.toggled.connect(lambda: self.radio_button_selected("ADC1", "adc"))
        self.adc_tab_options.adc2_button.toggled.connect(lambda: self.radio_button_selected("ADC2", "adc"))
        self.adc_tab_options.adc3_button.toggled.connect(lambda: self.radio_button_selected("ADC3", "adc"))

    def generic_button_connections(self):
        self.accel_tab_options.all_accel_button.toggled.connect(lambda: self.radio_button_selected("All", "accel"))
        self.gyro_tab_options.all_gyro_button.toggled.connect(lambda: self.radio_button_selected("All", "gyro"))
        self.adc_tab_options.all_adc_button.toggled.connect(lambda: self.radio_button_selected("All", "adc"))

    def radio_button_selected(self, radio_button, graph):  # pylint: disable=W0221
        if graph == "accel":
            self.accel_data_plot.show_trace(radio_button)
        elif graph == "gyro":
            self.gyro_data_plot.show_trace(radio_button)
        elif graph == "adc":
            self.adc_data_plot.show_trace(radio_button)
