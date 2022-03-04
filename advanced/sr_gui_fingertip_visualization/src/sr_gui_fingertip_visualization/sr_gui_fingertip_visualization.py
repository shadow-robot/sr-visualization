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
import rostopic
import sys

from python_qt_binding.QtCore import Qt, QTimer, QRectF
from python_qt_binding.QtGui import QPainter, QColor
from sr_gui_fingertip_visualization.dot_unit import DotUnitPST

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
    BiotacVisualizationTab,
    GenericTabLayout
)

from qt_gui.plugin import Plugin

from sr_utilities.hand_finder import HandFinder
from sr_hand.tactile_receiver import TactileReceiver
from sr_robot_msgs.msg import BiotacAll, ShadowPST


class SrFingertipVisualizer(Plugin):
    TITLE = "Fingertip Visualizer"

    def __init__(self, context):
        super().__init__(context)

        self.detect_hand_and_tactile_type()
        self.context = context
        self.init_ui()

    def detect_hand_and_tactile_type(self):
        type_right = rostopic.get_topic_type("/rh/tactile")
        type_left = rostopic.get_topic_type("/lh/tactile")

        self._tactile_topics = dict()

        self._hand_ids = [i[1].split('/')[1] for i in [type_right, type_left] if i[1]]
        self._t = [i[0].split('/')[1] for i in [type_right, type_left] if i[1]]
        
        for i, id in enumerate(self._hand_ids):
            self._tactile_topics[id] = self._t[i]

    def init_ui(self):
        self._widget = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self._widget.setObjectName(self.TITLE)
        self._widget.setWindowTitle(self.TITLE)        

        self._widget.setLayout(self.main_layout)
        self.fill_layout()

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
        tactile_topic = self._tactile_topics[hand_id]

        if tab_name == "Visualizer":
            if tactile_topic == "ShadowPST":
                self.tab_created = PSTVisualizationTab(tab_name, self._widget)
                self.tab_container.addTab(self.tab_created, tab_name)
            elif tactile_topic == "BiotacAll":
                self.tab_created = BiotacVisualizationTab(tab_name, self._widget)
                self.tab_container.addTab(self.tab_created, tab_name)

        elif tab_name == "Graphs":
            self.tab_created = BiotacVisualizationTab(tab_name)

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
    app = QGuiApplication(sys.argv)
    data_visualiser_gui = SrDataVisualizer(None)
    data_visualiser_gui._widget.show()
    sys.exit(app.exec_())
