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

import sys
import rospy
from sensor_msgs.msg import JointState
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget,
    QApplication,
    QTabWidget,
    QVBoxLayout,
    QPushButton,
    QMessageBox,
    QLabel,
    QComboBox,
    QHBoxLayout
)
from sr_data_visualization.data_plot import GenericDataPlot
from sr_data_visualization.data_tab import (
    JointStatesDataTab,
    ControlLoopsDataTab,
    MotorStats1DataTab,
    MotorStats2DataTab,
    PalmExtrasDataTab
)


class SrDataVisualizer(Plugin):
    TITLE = "Data Visualizer"
    PREFIXES = ['rh_', 'lh_']

    def __init__(self, context):
        super().__init__(context)
        self.joint_prefixes = []
        self.hand_id_selection = QComboBox()
        self.info_button_and_hand_selection_layout = QHBoxLayout()
        self.tab_index = 0
        self.joint_prefix = None
        self.hand_joints = None
        self.information_btn = None
        self.tab_container = None
        self.tab_created = None
        self.context = context
        self.init_ui()

    def _detect_hand_id_and_joints(self):
        self.hand_joints = None
        try:
            joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=1)
            for prefix in self.PREFIXES:
                for joint in joint_states_msg.name:
                    if prefix in joint:
                        self.joint_prefixes.append(prefix)
                        break
            joints = [joint for joint in joint_states_msg.name if self.joint_prefixes[0] in joint]
            self.hand_joints = {self.joint_prefixes[0][:-1]: joints}
        except (rospy.exceptions.ROSException, IndexError):
            rospy.logwarn("No hand connected or ROS bag is not playing")

        return self.joint_prefixes and self.hand_joints

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
        self.information_btn.clicked.connect(self.display_information)
        self.information_btn.setFixedSize(100, 20)
        self.tab_container = QTabWidget()

        if not self._detect_hand_id_and_joints():
            self.layout.addWidget(QLabel("No hand connected or ROS bag is not playing"), alignment=Qt.AlignCenter)
            return

        labels = []
        for prefix in self.joint_prefixes:
            labels.append(prefix[:-1])
        self.hand_id_selection.addItems(labels)
        self.hand_id_selection.currentIndexChanged.connect(self.combobox_action_hand_id_selection)
        self.hand_id_selection.setFixedSize(50, 20)

        self.info_button_and_hand_selection_layout.addWidget(QLabel("Hand ID:"), alignment=Qt.AlignRight)
        self.info_button_and_hand_selection_layout.addWidget(self.hand_id_selection)
        self.info_button_and_hand_selection_layout.addWidget(self.information_btn)
        self.layout.addLayout(self.info_button_and_hand_selection_layout)

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
                                                  self.joint_prefixes[0], parent=self.tab_container)
        elif tab_name == "Control Loops":
            self.tab_created = ControlLoopsDataTab(tab_name, self.hand_joints,
                                                   self.joint_prefixes[0], parent=self.tab_container)
        elif tab_name == "Motor Stats 1":
            self.tab_created = MotorStats1DataTab(tab_name, self.hand_joints,
                                                  self.joint_prefixes[0], parent=self.tab_container)
        elif tab_name == "Motor Stats 2":
            self.tab_created = MotorStats2DataTab(tab_name, self.hand_joints,
                                                  self.joint_prefixes[0], parent=self.tab_container)
        elif tab_name == "Palm Extras":
            self.tab_created = PalmExtrasDataTab(tab_name, self.hand_joints,
                                                 self.joint_prefixes[0], parent=self.tab_container)

        self.tab_container.addTab(self.tab_created, tab_name)

    def tab_changed(self, index):
        self.tab_index = index
        side = self.hand_id_selection.currentText()
        for tab in range(self.tab_container.count()):
            graphs = self.tab_container.widget(tab).findChildren(GenericDataPlot)
            self.tab_container.widget(tab).change_side(side)
            if tab is not index:
                for graph in graphs:
                    graph.plot_data(False, side)
            else:
                for graph in graphs:
                    graph.plot_data(True, side)

    def display_information(self, message):
        # pylint: disable=R0201
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

    def combobox_action_hand_id_selection(self):
        side = self.hand_id_selection.currentText()

        for tab in range(self.tab_container.count()):
            graphs = self.tab_container.widget(tab).findChildren(GenericDataPlot)
            self.tab_container.widget(tab).change_side(side)
            if tab is not self.tab_index:
                for graph in graphs:
                    graph.plot_data(False, side)
            else:
                for graph in graphs:
                    graph.plot_data(True, side, new_sub=False)

    def shutdown_plugin(self):
        for tab in range(self.tab_container.count()):
            graphs = self.tab_container.widget(tab).findChildren(GenericDataPlot)
            for graph in graphs:
                graph.plot_data(False, self.hand_id_selection.currentText())


if __name__ == "__main__":
    rospy.init_node("sr_data_visualizer")
    app = QApplication(sys.argv)
    data_visualiser_gui = SrDataVisualizer(None)
    data_visualiser_gui._widget.show()  # pylint: disable=W0212
    sys.exit(app.exec_())
