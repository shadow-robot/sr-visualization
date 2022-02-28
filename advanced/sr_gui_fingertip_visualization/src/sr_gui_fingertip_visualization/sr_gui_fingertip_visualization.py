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

from python_qt_binding.QtCore import Qt

from python_qt_binding.QtWidgets import (
    QWidget,
    QApplication,
    QTabWidget,
    QVBoxLayout,
    QBoxLayout,
    QHBoxLayout,
    QPushButton,
    QComboBox,
    QMessageBox
)

from sr_gui_fingertip_visualization.tab_layouts import (
    PSTVisualizationTab,
    BiotacVisualizationTab
)

from rqt_gui_py.plugin import Plugin

from sr_utilities.hand_finder import HandFinder
from sr_hand.tactile_receiver import TactileReceiver


class SrFingertipVisualizer(Plugin):
    TITLE = "Fingertip Visualizer"

    def __init__(self, context):
        super().__init__(context)

        self.detect_hand_and_tactile_type()
        self.context = context
        self.init_ui()

    def detect_hand_and_tactile_type(self):
        self._hand_ids = list(id.strip('_') for id in HandFinder().get_hand_parameters().joint_prefix.values())
        self._tactile_types = dict()
        for id in self._hand_ids:
            self._tactile_types[id] = TactileReceiver(id).find_tactile_type()

    def init_ui(self):
        self._widget = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self._widget.setObjectName(self.TITLE)
        self._widget.setWindowTitle(self.TITLE)

        self.fill_layout()

        self._widget.setLayout(self.main_layout)

        if __name__ != "__main__":
            self.context.add_widget(self._widget)

    def fill_layout(self):

        # Create info button on the top right of the gui
        self.information_btn = QPushButton("Info")
        self.main_layout.addWidget(self.information_btn, alignment=Qt.AlignRight)       

        # Initialize tabs
        self.tab_container = QTabWidget()
        self.main_layout.addWidget(self.tab_container)        

        # Create tabs
        self.create_tab("Visualizer")
        #self.create_tab("Graphs")

        self.tab_container.currentChanged.connect(self.tab_changed)
        self.information_btn.clicked.connect(self.display_information)

    def create_tab(self, tab_name):
        
        hand_id = self._hand_ids[0]
        tactile_type = self._tactile_types[hand_id]

        rospy.logwarn(hand_id)
        rospy.logwarn(tactile_type)

        if tab_name == "Visualizer":
            if tactile_type == "PST":
                self.tab_created = PSTVisualizationTab(tab_name, parent=self.tab_container)
            elif tactile_type == "biotac":
                self.tab_created = BiotacVisualizationTab(tab_name, parent=self.tab_container)

        elif tab_name == "Graphs":
            self.tab_created = BiotacVisualizationTab(tab_name, parent=self.tab_container)

        self.tab_container.addTab(self.tab_created, tab_name)

    def tab_changed(self, index):
        pass

    def display_information(self, message):
        message = "To be defined"
        msg = QMessageBox()
        msg.setWindowTitle("Information")
        msg.setIcon(QMessageBox().Information)
        msg.setText(message)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def shutdown_plugin(self):
        pass


if __name__ == "__main__":
    rospy.init_node("sr_data_visualizer")
    app = QApplication(sys.argv)
    data_visualiser_gui = SrDataVisualizer(None)
    data_visualiser_gui._widget.show()
    sys.exit(app.exec_())
