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

import random
import numpy as np
import rospy
import sys

from qtpy.QtWidgets import QFrame
from qtpy.QtGui import QPen, QBrush
from qtpy.QtCore import QSize, Qt
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QAction, QTabWidget, QGridLayout, QCheckBox
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
        self._joint_prefix = next(iter(list(self._hand_parameters.joint_prefix.values())))
        self._hand_joints = self._hand_finder.get_hand_joints()

        self.init_ui()

    def init_ui(self):
        title = self.TITLE
        self.setWindowTitle(title)

        self.init_main_widget()
        # self.tab_widget = CreateCentralTabs(self)
        
        self.show()

    def init_main_widget(self):
        self.layout = QGridLayout(self)
        
        # Initialize tab screen
        self.tab_widget = QTabWidget(self)
        self.tab_widget.resize(300,200)
        
        # Create tabs
        self.create_all_tab("Joint States")
        self.create_all_tab("Joint States 2")
        # self.create_all_tab("ANOTHER TAB2")
      
        # Add tabs to widget
        self.layout.addWidget(self.tab_widget)
        self.setLayout(self.layout)

        self.setCentralWidget(self.tab_widget)
    
    def create_all_tab(self, tab_name):
        self.tab_created = QWidget()
        self.tab_created.layout = QGridLayout(self)
    


        # for x in range(0,5):
        #     for y in range 
        # for joint in self._hand_joints:
        if tab_name == "Joint States":
            THJ1 = self.create_joint_graph("rh_THJ1")
            FFJ1 = self.create_joint_graph("rh_FFJ1")
            MFJ1 = self.create_joint_graph("rh_MFJ1")
            RFJ1 = self.create_joint_graph("rh_RFJ1")

            self.tab_created.layout.addWidget(THJ1, 0, 0)
            self.tab_created.layout.addWidget(FFJ1, 0, 1)
            self.tab_created.layout.addWidget(MFJ1, 0, 2)
            self.tab_created.layout.addWidget(RFJ1, 0, 3)

        if tab_name == "Joint States 2":
            THJ2 = self.create_joint_graph("rh_THJ2")
            FFJ2 = self.create_joint_graph("rh_FFJ2")
            MFJ2 = self.create_joint_graph("rh_MFJ2")
            RFJ2 = self.create_joint_graph("rh_RFJ2")

            self.tab_created.layout.addWidget(THJ2, 0, 0)
            self.tab_created.layout.addWidget(FFJ2, 0, 1)
            self.tab_created.layout.addWidget(MFJ2, 0, 2)
            self.tab_created.layout.addWidget(RFJ2, 0, 3)
            
        self.tab_widget.addTab(self.tab_created, tab_name)
        self.tab_created.setLayout(self.tab_created.layout)


    def create_joint_graph(self, joint):
        joint_graph_widget = QWidget()
        joint_graph_widget.layout = QGridLayout(self)
        joint_check_box = QCheckBox(joint[3:])
        joint_plot = DataPlot(joint)
        joint_graph_widget.layout.addWidget(joint_check_box, 0, 0)
        joint_graph_widget.layout.addWidget(joint_plot, 1, 0)
        joint_graph_widget.setLayout(joint_graph_widget.layout)

        return joint_graph_widget


class DataPlot(QwtPlot):
    def __init__(self, joint_name, unattended=False):
        QwtPlot.__init__(self)

        self._joint_name =  joint_name
        
        self.setCanvasBackground(Qt.white)
        # not sure if autoscale is a good idea or not?
        # https://pythonhosted.org/python-qwt/reference/plot.html
        self.axisAutoScale(QwtPlot.xBottom)
        self.axisAutoScale(QwtPlot.yLeft)
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

        mY = QwtPlotMarker()
        mY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        mY.setLineStyle(QwtPlotMarker.HLine)
        mY.setYValue(0.0)
        mY.attach(self)

        self.setAxisTitle(QwtPlot.xBottom, "Time (seconds)")

        self.ready_for_new = True
        self.joint_state_data = {
            'Position': 0.0,
            'Effort': 0.0,
            'Velocity': 0.0
        }

        self._joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self._joint_state_cb, queue_size=1)

        self.startTimer(10 if unattended else 50)
        self.phase = 0.0

    def _joint_state_cb(self, joint_state):
        for name, position, velocity, effort in zip(joint_state.name, joint_state.position,
                                                        joint_state.velocity, joint_state.effort):
            if name == self._joint_name:
                self.joint_state_data['Position'] = position
                self.joint_state_data['Effort'] = effort
                self.joint_state_data['Velocity'] = velocity
        # rospy.logerr(str(self.joint_state_data))

    def alignScales(self):
        self.canvas().setFrameStyle(QFrame.Box | QFrame.Plain)
        self.canvas().setLineWidth(1)
        for axis_id in QwtPlot.AXES:
            scaleWidget = self.axisWidget(axis_id)
            if scaleWidget:
                scaleWidget.setMargin(0)
            scaleDraw = self.axisScaleDraw(axis_id)
            if scaleDraw:
                scaleDraw.enableComponent(QwtAbstractScaleDraw.Backbone, False)

    def timerEvent(self, e):
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
        self.replot()


if __name__ == "__main__":
    from qwt import tests
    rospy.init_node('trial_plots', anonymous=True)
    # app = tests.test_widget(DataVisualizer, options=False)

    app = QApplication(sys.argv)
    ex = DataVisualizer()
    sys.exit(app.exec_())
    # app = tests.test_widget(DataPlot, size=(500, 300))