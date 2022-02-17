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
    QWidget,
    QGridLayout,
    QVBoxLayout,
)

from joint_graph_widget import JointGraph
from data_plot import JointStatesDataPlot, ControlLoopsDataPlot
from tab_options import JointStatesTabOptions, ControlLoopsTabOptions

from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState

class GenericDataTab(QWidget):
    """
        Creates the joint graph widget
    """
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        QWidget.__init__(self, parent=parent)

        self.tab_name = tab_name
        self.hand_joints = hand_joints
        self.joint_prefix = joint_prefix
        self.init_ui()
        self.create_full_tab()
        self.optional_button_connections()
        self.generic_button_connections()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_full_tab(self):
        self.create_tab_options()
        self.layout.addWidget(self.tab_options)

        self.graphs_layout = QGridLayout()
        self.create_all_graphs()

        self.layout.addLayout(self.graphs_layout)

    def create_tab_options(self):
        pass

    def create_all_graphs(self):
        pass

    def optional_button_connections(self):
        pass

    def generic_button_connections(self):
        self.tab_options.all_button.toggled.connect(lambda: self.radio_button_selected("All"))
        self.tab_options.show_seleted_button.clicked.connect(lambda: self.check_button_selected("Selection"))
        self.tab_options.reset_button.clicked.connect(lambda: self.check_button_selected("All"))

    def radio_button_selected(self, radio_button):
        for child in self.findChildren(JointGraph):
            child.joint_plot.turn_off_trace(radio_button)

    def check_button_selected(self, selection_type):
        index_to_display = 0
        max_no_columns = 4
        # self.temp_layout = QGridLayout()
        for child in self.findChildren(JointGraph):
            if selection_type == "Selection":
                if not child.joint_check_box.isChecked():
                    child.hide()
                else:
                    self.graphs_layout.addWidget(child,
                                                 index_to_display // max_no_columns,
                                                 index_to_display % max_no_columns)
                    index_to_display += 1
            elif selection_type == "All":
                if child.joint_check_box.isChecked():
                    child.joint_check_box.setCheckState(False)
                    self.graphs_layout.addWidget(child, child.initial_row, child.initial_column)
                else:
                    child.show()


class JointStatesDataTab(GenericDataTab):
    """
        Creates the joint graph widget
    """
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        super().__init__(tab_name, hand_joints, joint_prefix, parent)

    def create_tab_options(self):
        self.tab_options = JointStatesTabOptions(self.tab_name)

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

    def optional_button_connections(self):
        self.tab_options.position_button.toggled.connect(lambda: self.radio_button_selected("Position"))
        self.tab_options.velocity_button.toggled.connect(lambda: self.radio_button_selected("Velocity"))
        self.tab_options.effort_button.toggled.connect(lambda: self.radio_button_selected("Effort"))


class ControlLoopsDataTab(GenericDataTab):
    """
        Creates the joint graph widget
    """
    def __init__(self, tab_name, hand_joints, joint_prefix, parent=None):
        super().__init__(tab_name, hand_joints, joint_prefix, parent)

    def create_tab_options(self):
        self.tab_options = ControlLoopsTabOptions(self.tab_name)

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
                    topic_name = '/sh_' + joint.lower() + '_position_controller/state'
                    topic_type = JointControllerState
                    data_plot = ControlLoopsDataPlot(joint, topic_name, topic_type)
                    graph = JointGraph(joint, data_plot, row, column)
                    self.graphs_layout.addWidget(graph, row, column)
                    row += 1

    def optional_button_connections(self):
        self.tab_options.setpoint_button.toggled.connect(lambda: self.radio_button_selected("Set Point"))
        self.tab_options.input_button.toggled.connect(lambda: self.radio_button_selected("Input"))
        self.tab_options.dinputdt_button.toggled.connect(lambda: self.radio_button_selected("dInput/dt"))
        self.tab_options.error_button.toggled.connect(lambda: self.radio_button_selected("Error"))
        self.tab_options.output_button.toggled.connect(lambda: self.radio_button_selected("Output"))
