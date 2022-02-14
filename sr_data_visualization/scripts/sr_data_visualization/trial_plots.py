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
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QAction, QTabWidget, QGridLayout, QCheckBox, QRadioButton, QVBoxLayout
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
    SIZE = (1000, 500)

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
        self.layout = QGridLayout(self)

        # Initialize tab screen
        self.tab_widget = QTabWidget(self)
        self.tab_widget.resize(300, 200)

        # Create tabs
        self.create_tab("Joint States")
        # self.create_all_tab("Joint States 2")
        # self.create_all_tab("Joint States 3")

        # Add tabs to widget
        self.layout.addWidget(self.tab_widget)
        self.setLayout(self.layout)

        self.setCentralWidget(self.tab_widget)

        self.tab_widget.currentChanged.connect(self.tab_changed)


    def create_tab(self, tab_name):
        self.tab_created = DataTab(tab_name, self.hand_joints, self.joint_prefix)

        self.tab_widget.addTab(self.tab_created.full_tab, tab_name)
        self.tab_created.setLayout(self.tab_created.full_tab.layout)

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
        # self.options = self.create_tab_options()
        self.graphs = self.create_all_graphs(hand_joints, joint_prefix)
        self.full_tab = self.create_full_tab()

    def create_full_tab(self):
        full_tab = QWidget()
        full_tab.layout = QVBoxLayout(self)

        # full_tab.layout.addWidget(self.options, 0, 0)
        full_tab.layout.addWidget(self.graphs)
        full_tab.setLayout(full_tab.layout)

        return full_tab

    # def create_tab_options(self):
    #   layout = QGridLayout()
    #   b1 = QRadioButton("Button1")
    #   b1.setChecked(True)
    # #   self.b1.toggled.connect(lambda:self.btnstate(self.b1))
    #   layout.addWidget(b1)
		
    #   b2 = QRadioButton("Button2")
    # #   self.b2.toggled.connect(lambda:self.btnstate(self.b2))

    #   layout.addWidget(b2)
      

    #   return layout

    def create_all_graphs(self, hand_joints, joint_prefix):
        graphs = QWidget()
        graphs.layout = QGridLayout(self)

        # for x in range(0,5):
        #     for y in range
        joints = {
            0: [],
            1: [],
            2: [],
            3: [],
            4: []
        }
        for joint in hand_joints[joint_prefix[:-1]]:
            if "_THJ" in joint:
                joints[0].append(joint)
            if "_FFJ" in joint:
                joints[1].append(joint)
            if "_MFJ" in joint:
                joints[2].append(joint)
            if "_RFJ" in joint:
                joints[3].append(joint)
            if "_LFJ" in joint:
                joints[4].append(joint)

        if self.tab_name == "Joint States":
            for collumn, joint_names in joints.items():
                row = 0
                for joint in joint_names:
                    joint_graph = JointGraph(joint)

                    graphs.layout.addWidget(joint_graph.joint_widget, row, collumn)
                    row += 1

        if self.tab_name == "Joint States 2":
            THJ2 = JointGraph("rh_THJ2")
            FFJ2 = JointGraph("rh_FFJ2")
            MFJ2 = JointGraph("rh_MFJ2")
            RFJ2 = JointGraph("rh_RFJ2")

            graphs.layout.addWidget(THJ2.joint_widget, 0, 0)
            graphs.layout.addWidget(FFJ2.joint_widget, 0, 1)
            graphs.layout.addWidget(MFJ2.joint_widget, 0, 2)
            graphs.layout.addWidget(RFJ2.joint_widget, 0, 3)

        if self.tab_name == "Joint States 3":
            THJ2 = JointGraph("rh_THJ2")

            graphs.layout.addWidget(THJ2.joint_widget, 0, 0)

        return graphs


class JointGraph(QWidget):
    """
        Creates the joint graph widget
    """
    def __init__(self, joint_name):
        QWidget.__init__(self)

        self.joint_name = joint_name
        self.joint_widget = self._create_joint_graph_widget()

    def _create_joint_graph_widget(self):
        joint_graph_widget = QWidget()
        joint_graph_widget.layout = QGridLayout(self)
        self.joint_check_box = QCheckBox(self.joint_name[3:])
        self.joint_plot = DataPlot(self.joint_name)
        joint_graph_widget.layout.addWidget(self.joint_check_box, 0, 0)
        joint_graph_widget.layout.addWidget(self.joint_plot, 1, 0)
        joint_graph_widget.setLayout(joint_graph_widget.layout)

        return joint_graph_widget

    def plot_data(self, plot):
        self.joint_plot.plot_data(plot)


class DataPlot(QwtPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, unattended=False):
        QwtPlot.__init__(self)

        self._joint_name = joint_name
        self.unattended = unattended

        self.setCanvasBackground(Qt.white)
        # not sure if autoscale is a good idea or not?
        # https://pythonhosted.org/python-qwt/reference/plot.html
        # self.axisAutoScale(QwtPlot.xBottom)
        # self.axisAutoScale(QwtPlot.yLeft)
        # self.alignScales()

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

        # mY = QwtPlotMarker()
        # mY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        # mY.setLineStyle(QwtPlotMarker.HLine)
        # mY.setYValue(0.0)
        # mY.attach(self)

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
        # data moves from left to right:
        # shift data array right and assign new value data[0]
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

        self.position_plot.setData(self.x, self.position_data)
        self.effort_plot.setData(self.x, self.effort_data)
        self.velocity_plot.setData(self.x, self.velocity_data)

        # rospy.logerr(str(len(self.effort_data)) + "\n" + str(self.effort_data))
        self.replot()

    def plot_data(self, plot):
        if plot:
            self.timer.start()
        else:
            self.timer.stop()


if __name__ == "__main__":
    from qwt import tests
    rospy.init_node('trial_plots', anonymous=True)

    app = QApplication(sys.argv)
    ex = DataVisualizer()
    sys.exit(app.exec_())
