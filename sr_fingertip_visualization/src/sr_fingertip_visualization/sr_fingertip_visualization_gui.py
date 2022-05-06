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
import rostopic
from qt_gui.plugin import Plugin

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QApplication,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QPushButton,
    QMessageBox,
    QLabel
)
from sr_fingertip_visualization.tab_layouts_visual import VisualizationTab
from sr_fingertip_visualization.tab_layouts_graph import GraphTab


class SrFingertipVisualizer(Plugin):
    TITLE = "Fingertip Visualizer"

    def __init__(self, context):
        super().__init__(context)
        self.context = context
        self.tab_container = QTabWidget()
        self._widget = QWidget()
        self.main_layout = QVBoxLayout()
        self._tactile_topics = {}
        self._detect_hand_and_tactile_type()
        self._init_ui()

    def _detect_hand_and_tactile_type(self):
        try:
            type_right = rostopic.get_topic_type("/rh/tactile")
            type_left = rostopic.get_topic_type("/lh/tactile")

            for topic_type in [type_right, type_left]:
                if topic_type[0]:
                    rospy.wait_for_message(topic_type[1], rostopic.get_topic_class(topic_type[1])[0], timeout=1)

            self._hand_ids = [topic_data[1].split('/')[1] for topic_data in [type_right, type_left] if topic_data[1]]
            self._types = [topic_data[0].split('/')[1] for topic_data in [type_right, type_left] if topic_data[1]]
            self._tactile_topics = dict(zip(self._hand_ids, self._types))

        except (rospy.exceptions.ROSException, AttributeError):
            msg = "No available messages on tactile topic!"
            rospy.logwarn(msg)

    def _init_ui(self):
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self._widget.setObjectName(self.TITLE)
        self._widget.setWindowTitle(self.TITLE)
        self._widget.setLayout(self.main_layout)
        self.fill_layout()

        if __name__ != "__main__":
            self.context.add_widget(self._widget)

    def fill_layout(self):
        information_btn = QPushButton("Info")
        self.main_layout.addWidget(information_btn, alignment=Qt.AlignRight)

        self.tab_container.currentChanged.connect(self.tab_changed)
        information_btn.clicked.connect(display_information)

        if len(self._tactile_topics) == 0:
            label = QLabel("No tactiles", parent=self.tab_container)
            label.setSizePolicy(1, 1)
            self.main_layout.addWidget(label, alignment=Qt.AlignCenter)
        else:
            self.create_tab("Visualizer")
            self.create_tab("Graphs")
            self.main_layout.addWidget(self.tab_container)

    def create_tab(self, tab_name):
        if tab_name == "Visualizer":
            tab = VisualizationTab(self._tactile_topics)
            self.tab_container.addTab(tab, tab_name)

        elif tab_name == "Graphs":
            tab = GraphTab(self._tactile_topics)
            self.tab_container.addTab(tab, tab_name)

    def tab_changed(self, index):
        current_tab = self.tab_container.widget(index)
        for i in range(self.tab_container.count()):
            indexed_widget = self.tab_container.widget(i)
            tactile_widgets = indexed_widget.get_tactile_widgets()
            for tactile_widget in tactile_widgets.values():
                for fingertip_widget in tactile_widget.get_finger_widgets().values():
                    if indexed_widget is not current_tab:
                        fingertip_widget.stop_timer_and_subscriber()
                    elif fingertip_widget.isChecked():
                        fingertip_widget.start_timer_and_subscriber()

    def shutdown_plugin(self):
        for i in range(self.tab_container.count()):
            tactile_widgets = self.tab_container.widget(i).get_tactile_widgets()
            for tactile_widget in tactile_widgets.values():
                for fingertip_widget in tactile_widget.get_finger_widgets().values():
                    fingertip_widget.stop_timer_and_subscriber()

    def get_widget(self):
        return self._widget

def display_information(message):
    message = "This plugin is used to display data coming from the tactile sensors" + "\n" + \
                "of the Dexterous Hand. There are 2 tabs - Visualizer and Graphs." + "\n" + \
                "As a user you can select which hands and corresponding sensors you would like " + \
                "to inspect by selecting the HandID." + "\n" + "Selecting a specific finger will enable " + \
                "or disable the refreshing." + "\n" + "You have also the possibility to present only selected " + \
                "fingers by pressing the Show selected button or bring back all of the fingers" + "\n" + \
                "to the tab by pressing the Show all button." + "\n" + \
                "The Visualizer tab respresents the data in form of tactile points changing " + \
                "their colours based on the value coming from the sensors." + "\n" + \
                "In case of a Dexterous Hand equiped with Biotacs as tactile sensors" + "\n" + \
                "there is also a button which will allow you to switch the visual representation mode " + \
                "of the tactile points between electrodes" + "\n" + "or pac values coming from the sensor." + "\n" + \
                "The Graphs tab respresents the data in form of plots for " + \
                "all of the data coming from the sensors." + "\n" + \
                "Ticking the corresponding checkbox for the datatype will either add" + "\n" + \
                "or remove the plot from the graph of the finger."
    msg = QMessageBox()
    msg.setWindowTitle("Information")
    msg.setIcon(QMessageBox().Information)
    msg.setText(message)
    msg.setStandardButtons(QMessageBox.Ok)
    msg.exec_()


if __name__ == "__main__":
    rospy.init_node("sr_fingertip_visualizer")
    app = QApplication(sys.argv)
    sr_fingertip_visualizer_gui = SrFingertipVisualizer(None)
    sr_fingertip_visualizer_gui.get_widget().show()
    sys.exit(app.exec_())
