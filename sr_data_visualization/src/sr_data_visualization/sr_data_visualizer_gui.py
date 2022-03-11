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

import rospy
import sys

from rqt_gui_py.plugin import Plugin
from sr_data_visualization.data_plot import GenericDataPlot
from sensor_msgs.msg import JointState
from python_qt_binding.QtCore import Qt

from python_qt_binding.QtWidgets import (
    QWidget,
    QApplication,
    QTabWidget,
    QVBoxLayout,
    QPushButton,
    QMessageBox,
    QLabel
)

from sr_data_visualization.data_tab import (
    GenericDataTab,
    JointStatesDataTab,
    ControlLoopsDataTab,
    MotorStats1DataTab,
    MotorStats2DataTab,
    PalmExtrasDataTab
)


class SrDataVisualizer(Plugin):
    TITLE = "Data Visualizer"

    def __init__(self, context):
        super().__init__(context)

        self.context = context
        self.init_ui()

    def _detect_hand_id_and_joints(self):
        self.joint_prefix = None
        self.hand_joints = None

        try:
            joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=1)
            self.joint_prefix = ([prefix for prefix in joint_states_msg.name if 'h' in prefix][0]).split("_")[0] + "_"
            joints = [joint for joint in joint_states_msg.name if self.joint_prefix in joint]
            self.hand_joints = {self.joint_prefix[:-1]: joints}
        except (rospy.exceptions.ROSException, IndexError):
            pass

        return self.joint_prefix and self.hand_joints

    def init_ui(self):
        self._widget = QWidget()
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self._widget.setObjectName(self.TITLE)
        self._widget.setWindowTitle(self.TITLE)

        self.fill_layout()

        self._widget.setLayout(self.layout)

        if __name__ != "__main__":
            self.context.add_widget(self._widget)

    def fill_layout(self):
        # Create info button on the top right of the gui
        self.information_btn = QPushButton("Info")
        self.layout.addWidget(self.information_btn, alignment=Qt.AlignRight)
        self.information_btn.clicked.connect(self.display_information)
        self.tab_container = QTabWidget()

        if not self._detect_hand_id_and_joints():
            self.layout.addWidget(QLabel("No hand connected or ROS bag is not playing"), alignment=Qt.AlignCenter)
            return

        # Initialize tabs
        self.layout.addWidget(self.tab_container)

        # Create tabs
        self.create_tab("Joint States")
        self.create_tab("Control Loops")
        self.create_tab("Motor Stats 1")
        self.create_tab("Motor Stats 2")
        self.create_tab("Palm Extras")

        self.tab_container.currentChanged.connect(self.tab_changed)

    def create_tab(self, tab_name):

        if tab_name == "Joint States":
            self.tab_created = JointStatesDataTab(tab_name, self.hand_joints,
                                                  self.joint_prefix, parent=self.tab_container)
        elif tab_name == "Control Loops":
            self.tab_created = ControlLoopsDataTab(tab_name, self.hand_joints,
                                                   self.joint_prefix, parent=self.tab_container)
        elif tab_name == "Motor Stats 1":
            self.tab_created = MotorStats1DataTab(tab_name, self.hand_joints,
                                                  self.joint_prefix, parent=self.tab_container)
        elif tab_name == "Motor Stats 2":
            self.tab_created = MotorStats2DataTab(tab_name, self.hand_joints,
                                                  self.joint_prefix, parent=self.tab_container)
        elif tab_name == "Palm Extras":
            self.tab_created = PalmExtrasDataTab(tab_name, self.hand_joints,
                                                 self.joint_prefix, parent=self.tab_container)

        self.tab_container.addTab(self.tab_created, tab_name)

    def tab_changed(self, index):
        for tab in range(self.tab_container.count()):
            graphs = self.tab_container.widget(tab).findChildren(GenericDataPlot)
            if tab is not index:
                for graph in graphs:
                    graph.plot_data(False)
            else:
                for graph in graphs:
                    graph.plot_data(True)

    def display_information(self, message):
        message = "This GUI shows all the data available for the Dexterous Hand.\n" + \
                  "In each tab, you can find information about:\n\n" + \
                  "Joint states (position, effort, velocity)\n\n" + \
                  "Control loops (setpoint, input, dinput/dt, output, error)\n\n" + \
                  "Motor stats (Strain Gauge Left, Strain Gauge Right, Measured PWM, " + \
                  "Measured Current, Measured Voltage, Measured Effort, Temperature, " + \
                  "Unfiltered position, Unfiltered force, Last Commanded Effort, Encoder Position)\n\n" + \
                  "Palm extras (Accelerometer, Gyro-meter, Analog inputs)\n\n" + \
                  "The radio buttons let you choose specific data to show or you can choose " + \
                  "“All” to see several graphs being displayed at the same time.\n\n" + \
                  "The check buttons next to each graph name allows you to show the graphs you select " + \
                  "in larger detail by checking the boxes of the graphs you want to see and clicking " + \
                  "“Show Selected”. To return to the full graph view click “Reset”.\n\n" + \
                  "NOTE: The more graphs that are on show on the data visualizer will be slower and " +  \
                  "can be unreadable. To be able to see a full scaled view of a specific data type, " + \
                  "toggle the correct radio button and check the graphs you want to see clearer."
        msg = QMessageBox()
        msg.setWindowTitle("Information")
        msg.setIcon(QMessageBox().Information)
        msg.setText(message)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def shutdown_plugin(self):
        for tab in range(self.tab_container.count()):
            graphs = self.tab_container.widget(tab).findChildren(GenericDataPlot)
            for graph in graphs:
                graph.plot_data(False)


if __name__ == "__main__":
    rospy.init_node("sr_data_visualizer")
    app = QApplication(sys.argv)
    data_visualiser_gui = SrDataVisualizer(None)
    data_visualiser_gui._widget.show()
    sys.exit(app.exec_())
