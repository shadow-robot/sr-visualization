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

from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QTabWidget,
)

from sr_utilities.hand_finder import HandFinder

from data_plot import JointStatesDataPlot, ControlLoopsDataPlot
from data_tab import JointStatesDataTab, ControlLoopsDataTab


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
        self.create_tab("Control Loops")

        self.tab_widget.currentChanged.connect(self.tab_changed)

        # Add tabs to widget
        self.setCentralWidget(self.tab_widget)

    def create_tab(self, tab_name):
        if tab_name == "Joint States":
            self.tab_created = JointStatesDataTab(tab_name, self.hand_joints, self.joint_prefix, parent=self)
        elif tab_name == "Control Loops":
            self.tab_created = ControlLoopsDataTab(tab_name, self.hand_joints, self.joint_prefix, parent=self)
        self.tab_widget.addTab(self.tab_created, tab_name)

    def tab_changed(self, index):
        for tab in range((self.tab_widget.count()-1)):
            if tab is not index:
                graphs = self.tab_widget.widget(tab).findChildren(JointStatesDataPlot)
                for graph in graphs:
                    graph.plot_data(False)
            else:
                graphs = self.tab_widget.widget(tab).findChildren(JointStatesDataPlot)
                for graph in graphs:
                    graph.plot_data(True)


if __name__ == "__main__":
    rospy.init_node('trial_plots', anonymous=True)

    app = QApplication(sys.argv)
    ex = DataVisualizer()
    sys.exit(app.exec_())
