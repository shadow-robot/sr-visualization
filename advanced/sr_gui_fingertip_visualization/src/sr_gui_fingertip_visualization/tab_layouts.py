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

import os
import rospkg
import rospy
import rostopic
from enum import Enum

from python_qt_binding.QtGui import QIcon, QColor, QPalette, QFontMetrics
from python_qt_binding.QtCore import Qt, QTimer

from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QGridLayout,
    QRadioButton,
    QHBoxLayout,
    QVBoxLayout,
    QLineEdit,
    QGroupBox,
    QProgressBar,
    QFormLayout,
    QLabel,
    QComboBox,
    QCheckBox,
    QSizePolicy,
    QSpacerItem,
    QLayout
)

from sr_gui_fingertip_visualization.dot_unit import DotUnitPST, DotUnitBiotacSPPlus, DotUnitBiotacSPMinus
from sr_robot_msgs.msg import ShadowPST, BiotacAll

from sr_utilities.hand_finder import HandFinder
from sr_hand.tactile_receiver import TactileReceiver


class GenericTabLayout(QWidget):
    def __init__(self, tab_name, parent):
        super().__init__()
        self.parent = parent

        type_right = rostopic.get_topic_type("/rh/tactile")
        type_left = rostopic.get_topic_type("/lh/tactile")
        self._hand_ids = [i[1].split('/')[1] for i in [type_right, type_left] if i[1]]

        self._fingers = ["ff", 'mf', 'rf', 'lf', 'th']
        self._finger_widgets = dict()
        self._finger_frame = dict()
        self._init_generic_layout()

    def _init_generic_layout(self):
        self.main_tab_layout = QVBoxLayout()
        self.main_tab_layout.setAlignment(Qt.AlignTop)
        self.main_tab_layout.setContentsMargins(0, 0, 0, 0)

        self.options_layout_groupbbox = QGroupBox("Options")
        self.options_layout = QHBoxLayout()

        self.hand_id_selection_layout = QFormLayout()
        self.hand_id_selection = QComboBox()
        self.hand_id_selection.addItems(self._hand_ids)
        self.hand_id_selection_layout.addRow(QLabel("Hand ID:"), self.hand_id_selection)

        self.finger_selection_label = QLabel("Finger selection:")
        self.finger_selection_show_selected_button = QPushButton("Show selected")
        self.finger_selection_show_selected_button.setSizePolicy(2, 2)
        self.finger_selection_show_all_button = QPushButton("Show all")
        self.finger_selection_show_all_button.setSizePolicy(2, 2)

        self.options_layout.addLayout(self.hand_id_selection_layout)
        self.options_layout.addStretch(1)
        self.options_layout.addWidget(self.finger_selection_label)
        self.options_layout.addWidget(self.finger_selection_show_selected_button)
        self.options_layout.addWidget(self.finger_selection_show_all_button)

        self.options_layout_groupbbox.setLayout(self.options_layout)
        self.main_tab_layout.addWidget(self.options_layout_groupbbox)

        rospy.logwarn("Inited generic layout elements")

    def _button_action_show_selected_fingers(self):
        self._selected_fingers = [finger for finger in self._fingers if self._finger_frame[finger].isChecked()]
        rospy.logwarn(self._selected_fingers)
        for finger in self._fingers:
            if finger in self._selected_fingers:
                self._finger_frame[finger].show()
            else:
                self._finger_frame[finger].hide()

    def _button_action_show_all(self):
        for finger in self._fingers:
            self._finger_frame[finger].show()
            self._finger_frame[finger].setChecked(True)

    def _initialize_and_start_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start(10)

    def _init_tactile_layout(self):
        raise NotImplementedError("The function _init_tactile_layout must be implemented")

    def initialize_subscribers(self):
        raise NotImplementedError("The function initialize_subscribers must be implemented")

    def _tactile_data_callback(self):
        raise NotImplementedError("The function _tactile_data_callback must be implemented")

    def _initialize_data_structure(self):
        raise NotImplementedError("The function _initialize_data_structure must be implemented")

    def _create_connections(self):
        raise NotImplementedError("The function _create_connections must be implemented")


class PSTVisualizationTab(GenericTabLayout):
    def __init__(self, tab_name, parent):
        super().__init__(tab_name, parent)
        self._initialize_data_structure()
        self.initialize_subscribers("rh")

        self._init_tactile_layout()
        self._create_connections()

        self._initialize_and_start_timer()

    def _initialize_data_structure(self):
        self._data_fields = ['pressure', 'temperature']
        self._data = dict()
        self.finger_widget = dict()
        self._finger_frame = dict()
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = 0

    def initialize_subscribers(self, side):
        self.sub = rospy.Subscriber('/{}/tactile'.format(side), ShadowPST, self._tactile_data_callback)

    def stop_subscribers(self, side):
        self.sub.unregister()

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            for data_field in self._data_fields:
                if data_field == "pressure":
                    self._data[finger][data_field] = data.pressure[i]
                elif data_field == "temperature":
                    self._data[finger][data_field] = data.temperature[i]

    def _init_tactile_layout(self):
        fingers_frame = QHBoxLayout()
        for finger in self._fingers:
            fingers_frame.addWidget(self.init_finger_widget(finger))
        self.main_tab_layout.addLayout(fingers_frame)
        rospy.logwarn("inited pst tactile layout")
        self.main_tab_layout.addLayout(fingers_frame)
        self.setLayout(self.main_tab_layout)

    def init_finger_widget(self, finger):
        self._finger_frame[finger] = QGroupBox(finger)
        self._finger_frame[finger].setCheckable(True)
        self._finger_frame[finger].setSizePolicy(1, 1)

        self.finger_widget[finger] = DotUnitPST(self._finger_frame[finger])

        layout = QHBoxLayout()
        layout.addWidget(self.finger_widget[finger])
        self._finger_frame[finger].setLayout(layout)

        return self._finger_frame[finger]

    def _create_connections(self):
        self.finger_selection_show_selected_button.clicked.connect(self._button_action_show_selected_fingers)
        self.finger_selection_show_all_button.clicked.connect(self._button_action_show_all)

    def timerEvent(self):
        for finger in self._fingers:
            if self._finger_frame[finger].isChecked():
                self.finger_widget[finger].update_data(self._data[finger])


class BiotacType(Enum):
    SP_PLUS = 0
    SP_MINUS = 1
    BLANK = 3


class BiotacVisualizationTab(GenericTabLayout):
    def __init__(self, tab_name, painter):
        super().__init__(tab_name, painter)

        self._version = "v2"  # Autodetect
        self._electrode_count = 0
        self._selected_fingers = []

        self._initialize_data_structure()
        self.initialize_subscriber('rh')

        self._init_tactile_layout()
        self._create_connections()

        self._initialize_and_start_timer()

    #  to detect if biotac is type sp+, sp-, or blank
    def _detect_biotac_type(self, finger):
        sum_of_electrode_values = sum(self._data[finger]['electrodes'])
        biotac_type = None
        if sum_of_electrode_values == 0:
            biotac_type = BiotacType.BLANK
        elif sum_of_electrode_values == self._electrode_count:
            biotac_type = BiotacType.SP_MINUS
        else:
            biotac_type = BiotacType.SP_PLUS
        return biotac_type

    def _create_connections(self):
        self.finger_selection_show_selected_button.clicked.connect(self._button_action_show_selected_fingers)
        self.finger_selection_show_all_button.clicked.connect(self._button_action_show_all)

    def _initialize_data_structure(self):
        self._data_fields = ['pac0', 'pac1', 'pac', 'pdc', 'tac', 'tdc', 'electrodes']
        self._data = dict()
        for finger in self._fingers:
            self._data[finger] = dict()
        #  Change to autodetect hand
        self._tactile_data_callback(rospy.wait_for_message('/rh/tactile', BiotacAll))

        self.coordinates = dict()
        self.coordinates['v1'] = dict()
        self.coordinates['v1']['sensing'] = dict()
        self.coordinates['v1']['sensing']['x'] = [6.45, 3.65, 3.65, 6.45, 3.65, 6.45, 0.00, 1.95, -1.95, 0.00,
                                                  -6.45, - 3.65, -3.65, -6.45, -3.65, -6.45, 0.00, 0.00, 0.00]
        self.coordinates['v1']['sensing']['y'] = [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38, 6.38, 8.38,
                                                  7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38, 18.38, 22.18]
        self.coordinates['v1']['excitation'] = dict()
        self.coordinates['v1']['excitation']['x'] = [6.45, 3.75, -3.75, -6.45]
        self.coordinates['v1']['excitation']['y'] = [12.48, 24.48, 24.48, 12.48]

        self.coordinates['v2'] = dict()
        self.coordinates['v2']['sensing'] = dict()
        self.coordinates['v2']['sensing']['x'] = [5.00, 3.65, 6.45, 4.40, 2.70, 6.45, 4.40, 1.50, 4.00, 4.50, -5.00,
                                                  -3.65, -6.45, -4.40, -2.70, -6.45, -4.40, -1.50, -4.00, -4.50, 0.00,
                                                  1.95, -1.95, 0.00]
        self.coordinates['v2']['sensing']['y'] = [4.38, 6.38, 10.78, 11.50, 14.50, 15.08, 16.00, 17.00, 19.00, 21.00,
                                                  4.38, 6.38, 10.78, 11.50, 14.50, 15.08, 16.00, 17.00, 19.00, 21.00,
                                                  7.38, 9.50, 9.50, 11.20]
        self.coordinates['v2']['excitation'] = dict()
        self.coordinates['v2']['excitation']['x'] = [5.30, 6.00, -5.30, -6.00]
        self.coordinates['v2']['excitation']['y'] = [9.00, 22.00, 9.00, 22.00]
        self._electrode_count = len(self.coordinates[self._version]['sensing']['x'])

    def _tactile_data_callback(self, data):
        for data_field in self._data_fields:
            for i, finger in enumerate(self._fingers):
                if data_field == "pac0":
                    self._data[finger][data_field] = data.tactiles[i].pac0
                elif data_field == "pac1":
                    self._data[finger][data_field] = data.tactiles[i].pac1
                elif data_field == "pac":
                    self._data[finger][data_field] = list(data.tactiles[i].pac)
                elif data_field == "pdc":
                    self._data[finger][data_field] = data.tactiles[i].pdc
                elif data_field == "tac":
                    self._data[finger][data_field] = data.tactiles[i].tac
                elif data_field == "tdc":
                    self._data[finger][data_field] = data.tactiles[i].tdc
                elif data_field == "electrodes":
                    self._data[finger][data_field] = list(data.tactiles[i].electrodes)

    def init_finger_widget(self, finger):
        self._finger_frame[finger] = QGroupBox(finger)
        self._finger_frame[finger].setCheckable(True)
        self._finger_frame[finger].setSizePolicy(1, 1)
        layout = QHBoxLayout()

        x_cords = self.coordinates[self._version]['sensing']['x']
        y_cords = self.coordinates[self._version]['sensing']['y']
        min_x = abs(min(self.coordinates[self._version]['sensing']['x']))
        min_y = abs(min(self.coordinates[self._version]['sensing']['y']))
        max_x = abs(max(self.coordinates[self._version]['sensing']['x']))
        max_y = abs(max(self.coordinates[self._version]['sensing']['y']))
        y_offset = 2

        if self._detect_biotac_type(finger) == BiotacType.SP_PLUS:
            self._finger_widgets[finger] = [QWidget()] * len(self.coordinates[self._version]['sensing']['x'])
            container_widget = QWidget()
            container_widget.setMinimumSize((max_x) * 50, ((max_y) * 19))
            for i, (x, y) in enumerate(zip(x_cords, y_cords)):
                self._finger_widgets[finger][i] = DotUnitBiotacSPPlus(i, container_widget)
                self._finger_widgets[finger][i].move((x + min_x) * 20, ((y - y_offset - min_y / 2) * 20))
            layout.addWidget(container_widget, alignment=Qt.AlignCenter)

        elif self._detect_biotac_type(finger) == BiotacType.SP_MINUS:
            self._finger_widgets[finger] = DotUnitBiotacSPMinus(self._finger_frame[finger])
            layout.addWidget(self._finger_widgets[finger], alignment=Qt.AlignCenter)

        elif self._detect_biotac_type(finger) == BiotacType.BLANK:
            self._finger_widgets[finger] = QLabel("No tactile sensor")
            layout.addWidget(self._finger_widgets[finger], alignment=Qt.AlignCenter)

        self._finger_frame[finger].setLayout(layout)
        return self._finger_frame[finger]

    def _init_tactile_layout(self):
        self.finger_complete_layout = QHBoxLayout()
        for finger in self._fingers:
            self.finger_complete_layout.addWidget(self.init_finger_widget(finger))
        self.main_tab_layout.addLayout(self.finger_complete_layout)
        self.setLayout(self.main_tab_layout)

    def initialize_subscriber(self, side):
        self.sub = rospy.Subscriber('/{}/tactile'.format(side), BiotacAll, self._tactile_data_callback)

    def stop_subscribers(self, side):
        self.sub.unregister()

    def timerEvent(self):
        for finger in self._selected_fingers:
            if self._finger_frame[finger].isChecked():
                if type(self._finger_widgets[finger]) == list:
                    for electrode in range(self._electrode_count):
                        if type(self._finger_widgets[finger][electrode]) == DotUnitBiotacSPPlus:
                            value = self._data[finger]['electrodes'][electrode]
                            self._finger_widgets[finger][electrode].update_data(value)
                elif type(self._finger_widgets[finger]) == DotUnitBiotacSPMinus:
                    self._finger_widgets[finger].update_data(self._data[finger])
