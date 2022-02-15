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

import random
import numpy as np
import rospy
import sys

from qtpy.QtWidgets import QFrame
from qtpy.QtGui import QPen, QBrush
from qtpy.QtCore import QSize, Qt, QTimer
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QPushButton,
    QWidget, QAction,
    QTabWidget,
    QGridLayout,
    QCheckBox,
    QRadioButton,
    QVBoxLayout,
    QHBoxLayout,
)
from qwt import (
    QwtPlot,
    QwtPlotMarker,
    QwtSymbol,
    QwtLegend,
    QwtPlotCurve,
    QwtAbstractScaleDraw,
)

from sensor_msgs.msg import JointState
from sr_utilities.hand_finder import HandFinder


class DataVisualizer(QMainWindow):
    TITLE = "Data Visualizer"
    # SIZE = (500, 500)

    def __init__(self):
        super(DataVisualizer, self).__init__()

        self._hand_finder = HandFinder()
        self._hand_parameters = self._hand_finder.get_hand_parameters()
        self.joint_prefix = next(iter(list(self._hand_parameters.joint_prefix.values())))
        self.hand_joints = self._hand_finder.get_hand_joints()

        self.init_ui()

    def init_ui(self):
        title = self.TITLE
        self.setWindowTitle(title)

        self.init_main_widget()

        self.show()

    def init_main_widget(self):
        # Initialize tab screen
        self.tab_widget = QTabWidget(self)

        # Create tabs
        self.create_tab("Joint States")

        self.tab_widget.currentChanged.connect(self.tab_changed)

        # Add tabs to widget
        self.setCentralWidget(self.tab_widget)

    def create_tab(self, tab_name):
        self.tab_created = DataTab(tab_name, self.hand_joints, self.joint_prefix)
        self.tab_widget.addTab(self.tab_created, tab_name)

    def tab_changed(self, index):
        for tab in range((self.tab_widget.count()-1)):
            if tab is not index:
                graphs = self.tab_widget.widget(tab).findChildren(DataPlot)
                for graph in graphs:
                    graph.plot_data(False)
            else:
                graphs = self.tab_widget.widget(tab).findChildren(DataPlot)
                for graph in graphs:
                    graph.plot_data(True)


class DataTab(QWidget):
    """
        Creates the joint graph widget
    """
    def __init__(self, tab_name, hand_joints, joint_prefix):
        QWidget.__init__(self)

        self.tab_name = tab_name
        self.hand_joints = hand_joints
        self.joint_prefix = joint_prefix
        self.init_ui()
        self.create_full_tab()
        self.button_connections()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_full_tab(self):
        self.tab_options = TabOptions(self.tab_name)
        self.layout.addWidget(self.tab_options)

        self.graphs_layout = QGridLayout()
        self.create_all_graphs()

        self.layout.addLayout(self.graphs_layout)

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
                graph = JointGraph(joint, row, column)
                graph.joint_check_box.stateChanged.connect(self.update_number)
                self.graphs_layout.addWidget(graph, row, column)
                row += 1

    def button_connections(self):
        self.tab_options.position_button.toggled.connect(lambda: self.radio_button_selected("position"))
        self.tab_options.velocity_button.toggled.connect(lambda: self.radio_button_selected("velocity"))
        self.tab_options.effort_button.toggled.connect(lambda: self.radio_button_selected("effort"))
        self.tab_options.all_button.toggled.connect(lambda: self.radio_button_selected("all"))
        self.tab_options.show_seleted_button.clicked.connect(lambda: self.check_button_selected("selection"))
        self.tab_options.reset_button.clicked.connect(lambda: self.check_button_selected("all"))

    def radio_button_selected(self, radio_button):
        for child in self.findChildren(JointGraph):
            child.joint_plot.turn_off_trace(radio_button)

    def check_button_selected(self, selection_type):
            index_to_display = 0
            max_no_columns = 4
            # self.temp_layout = QGridLayout()
            for child in self.findChildren(JointGraph):
                if selection_type == "selection":
                    if not child.joint_check_box.isChecked():
                        child.hide()
                    else:
                        self.graphs_layout.addWidget(child,
                                                     index_to_display // max_no_columns,
                                                     index_to_display % max_no_columns)
                        index_to_display += 1
                elif selection_type == "all":
                    if child.joint_check_box.isChecked():
                        child.joint_check_box.setCheckState(False)
                        self.graphs_layout.addWidget(child, child.initial_row, child.initial_column)
                    else:
                        child.show()


class TabOptions(QWidget):
    """
        Creates the options of filtering and selection for the tab
    """
    def __init__(self, tab_name):
        super().__init__()

        self.init_ui()
        self.create_tab_options()
        self.setLayout(self.layout)

    def init_ui(self):
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_tab_options(self):
        self.position_button = QRadioButton("Position")
        self.position_button.setObjectName("toggle_position")
        self.layout.addWidget(self.position_button)

        self.velocity_button = QRadioButton("Velocity")
        self.velocity_button.setObjectName("toggle_velocity")
        self.layout.addWidget(self.velocity_button)

        self.effort_button = QRadioButton("Effort")
        self.effort_button.setObjectName("toggle_effort")
        self.layout.addWidget(self.effort_button)

        self.all_button = QRadioButton("All")
        self.all_button.setObjectName("toggle_all")
        self.layout.addWidget(self.all_button)

        self.all_button.setChecked(True)

        self.show_seleted_button = QPushButton("Show Selected")
        self.show_seleted_button.setObjectName("show_seleted_button")
        self.layout.addWidget(self.show_seleted_button)

        self.reset_button = QPushButton("Reset")
        self.reset_button.setObjectName("reset_button")
        self.layout.addWidget(self.reset_button)


class JointGraph(QWidget):
    """
        Creates the joint graph widget
    """
    def __init__(self, joint_name, row, column):
        QWidget.__init__(self)

        self.joint_name = joint_name
        self.initial_row = row
        self.initial_column = column
        self.setObjectName(self.joint_name)
        self.init_ui()
        self._create_joint_graph_widget()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def _create_joint_graph_widget(self):
        self.joint_check_box = QCheckBox(self.joint_name)
        self.joint_plot = DataPlot(self.joint_name)
        self.layout.addWidget(self.joint_check_box)
        self.layout.addWidget(self.joint_plot)
        self.setLayout(self.layout)


class DataPlot(QwtPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, unattended=False):
        QwtPlot.__init__(self)

        self._joint_name = joint_name
        self.unattended = unattended

        self.setCanvasBackground(Qt.white)
        self.setMinimumSize(300, 150)

        # Initialize data
        self.x = np.arange(0.0, 100.1, 0.5)
        self.position_data = np.zeros(len(self.x), float)
        self.effort_data = np.zeros(len(self.x), float)
        self.velocity_data = np.zeros(len(self.x), float)

        # self.setTitle(self._joint_name)
        self.insertLegend(QwtLegend(), QwtPlot.TopLegend)

        # Create plots
        self.position_plot = QwtPlotCurve("Position")
        self.position_plot.attach(self)
        self.effort_plot = QwtPlotCurve("Effort")
        self.effort_plot.attach(self)
        self.velocity_plot = QwtPlotCurve("Velocity")
        self.velocity_plot.attach(self)

        self.position_plot.setPen(QPen(Qt.red))
        self.effort_plot.setPen(QPen(Qt.blue))
        self.velocity_plot.setPen(QPen(Qt.green))

        self.setAxisTitle(QwtPlot.xBottom, "Time (seconds)")

        self.ready_for_new = True
        self.joint_state_data = {
            'Position': 0.0,
            'Effort': 0.0,
            'Velocity': 0.0
        }

        self._joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self._joint_state_cb, queue_size=1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start(10 if self.unattended else 50)

    def _joint_state_cb(self, joint_state):
        for name, position, velocity, effort in zip(joint_state.name, joint_state.position,
                                                    joint_state.velocity, joint_state.effort):
            if name == self._joint_name:
                self.joint_state_data['Position'] = position
                self.joint_state_data['Effort'] = effort
                self.joint_state_data['Velocity'] = velocity

    def timerEvent(self):
        # Data moves from left to right:
        # Shift data array right and assign new value data[0]
        self.position_data = np.concatenate((self.position_data[:1], self.position_data[:-1]))
        self.position_data[0] = self.joint_state_data['Position']

        self.effort_data = np.concatenate((self.effort_data[:1], self.effort_data[:-1]))
        self.effort_data[0] = self.joint_state_data['Effort']

        self.velocity_data = np.concatenate((self.velocity_data[:1], self.velocity_data[:-1]))
        self.velocity_data[0] = self.joint_state_data['Velocity']

        self.position_plot.setData(self.x, self.position_data)
        self.effort_plot.setData(self.x, self.effort_data)
        self.velocity_plot.setData(self.x, self.velocity_data)
        self.replot()

    def plot_data(self, plot):
        if plot:
            self._joint_states_subscriber = rospy.Subscriber('joint_states',
                                                             JointState,
                                                             self._joint_state_cb,
                                                             queue_size=1)
            self.timer.start()
        else:
            self.timer.stop()
            self._joint_states_subscriber.unregister()

    def turn_off_trace(self, trace_name):
        if trace_name == "position":
            self.position_plot.attach(self)
            self.effort_plot.detach()
            self.velocity_plot.detach()
        if trace_name == "velocity":
            self.velocity_plot.attach(self)
            self.effort_plot.detach()
            self.position_plot.detach()
        if trace_name == "effort":
            self.effort_plot.attach(self)
            self.velocity_plot.detach()
            self.position_plot.detach()
        if trace_name == "all":
            self.position_plot.attach(self)
            self.velocity_plot.attach(self)
            self.effort_plot.attach(self)


if __name__ == "__main__":
    rospy.init_node('trial_plots', anonymous=True)

    app = QApplication(sys.argv)
    ex = DataVisualizer()
    sys.exit(app.exec_())
