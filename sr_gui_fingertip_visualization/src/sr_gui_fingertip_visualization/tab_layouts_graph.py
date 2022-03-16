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

from __future__ import absolute_import, division

import os
import rospy
import rospkg
from sr_robot_msgs.msg import ShadowPST, BiotacAll
from sr_gui_fingertip_visualization.generic_plots import GenericDataPlot
from sr_gui_fingertip_visualization.tab_layouts_generic import GenericGraphTab, GenericOptionBar, GenericOptionBar


from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QIcon, QColor
from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QFormLayout,
    QLabel,
    QComboBox,
    QCheckBox,
    QStackedLayout
)


from sr_gui_fingertip_visualization.finger_widgets_graphs import FingerWidgetGraphPST, FingerWidgetGraphBiotac


class PSTGraphTab(GenericGraphTab):
    def __init__(self, side, parent):
        super().__init__(side, parent)
        self._side = side
        self.parent = parent
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        self._data_fields = ['pressure', 'temperature']
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = list()

    def _init_tactile_layout(self):
        fingers_layout = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            fingers_layout.addWidget(self._init_finger_widget(finger))
        self.setLayout(fingers_layout)

    def _init_finger_widget(self, finger):
        self._finger_widgets[finger] = FingerWidgetGraphPST(self._side, finger, self)
        return self._finger_widgets[finger]


class BiotacGraphTab(GenericGraphTab):
    def __init__(self, side, parent):
        super().__init__(side, parent)
        self._side = side
        self.parent = parent
        self._init_tactile_layout()
       
    def _initialize_data_structure(self):
        self._data_fields = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = list()

    def _init_tactile_layout(self):
        fingers_layout = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            fingers_layout.addWidget(self._init_finger_widget(finger))
        self.setLayout(fingers_layout)

    def _init_finger_widget(self, finger):
        self._finger_widgets[finger] = FingerWidgetGraphBiotac(self._side, finger, self)
        return self._finger_widgets[finger]


class GraphTab(QWidget):
    def __init__(self, tactile_topics):
        super().__init__()
        self._tactile_topics = tactile_topics
        self._init_layout()

    def _init_layout(self):
        finger_layout = QVBoxLayout()
        self.stacked_layout = QStackedLayout()

        self.tactile_widgets = dict()
        for side, tactile_topic in self._tactile_topics.items():
            if tactile_topic == "ShadowPST":
                self.tactile_widgets[side] = PSTGraphTab(side, self)
            elif tactile_topic == "BiotacAll":
                self.tactile_widgets[side] = BiotacGraphTab(side, self)
            self.stacked_layout.addWidget(self.tactile_widgets[side])

        self._option_bar = GraphOptionBar(list(self._tactile_topics.keys()), childs=self.stacked_layout)
        
        finger_layout.addWidget(self._option_bar)        
        finger_layout.addLayout(self.stacked_layout)
        self.setLayout(finger_layout)

    def get_tactile_widgets(self):
        return self.tactile_widgets


class GraphOptionBar(GenericOptionBar):
    def __init__(self, hand_ids, childs):
        super().__init__(hand_ids, childs)
        self.init_layout()
        self.create_connections()

    def init_layout(self):
        super().init_layout()
        self.options_layout.addLayout(self.hand_id_selection_layout)
        self.options_layout.addStretch(1)
        self.options_layout.addWidget(self.finger_selection_label)
        self.options_layout.addWidget(self.finger_selection_show_selected_button)
        self.options_layout.addWidget(self.finger_selection_show_all_button)

        self.setLayout(self.options_layout)
        self._current_widget = self._childs.currentWidget()        
