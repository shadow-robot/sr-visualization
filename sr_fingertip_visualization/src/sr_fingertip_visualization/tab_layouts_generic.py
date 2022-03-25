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
from enum import Enum


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


class GenericGraphTab(QWidget):
    def __init__(self, side, parent):
        super().__init__(parent=parent)

        self._side = side
        self._CONST_FINGERS = ['ff', 'mf', 'rf', 'lf', 'th']
        self._CONST_DATA_FIELDS = ['pressure', 'temperature']
        self._data = dict()
        self._timer = QTimer()
        self._finger_widgets = dict()
        self._data_selection = dict()
        self._data_selection_checkboxes = dict()
        self._plot = dict()

        self._initialize_data_structure()

        ICON_DIR = os.path.join(rospkg.RosPack().get_path('sr_visualization_icons'), 'icons')
        self.ICONS = {
            'blue': QIcon(os.path.join(ICON_DIR, 'blue.png')),
            'red': QIcon(os.path.join(ICON_DIR, 'red.png')),
            'green': QIcon(os.path.join(ICON_DIR, 'green.png')),
            'magenta': QIcon(os.path.join(ICON_DIR, 'magenta.png')),
            'gray': QIcon(os.path.join(ICON_DIR, 'gray.png')),
            'cyan': QIcon(os.path.join(ICON_DIR, 'cyan.png'))
        }

        available_colors = list(self.ICONS.keys())
        self._legend_colors = dict()
        for i, data_field in enumerate(self._CONST_DATA_FIELDS):
            self._legend_colors[data_field] = dict()
            self._legend_colors[data_field]['icon'] = self.ICONS[available_colors[i]]
            self._legend_colors[data_field]['plot_color'] = QColor(available_colors[i])

    def _initialize_data_structure(self):
        raise NotImplementedError("The function _initialize_data_structure must be implemented")

    def start_timer_and_subscriber(self):
        raise NotImplementedError("The function start_timer_and_subscriber must be implemented")

    def get_finger_widgets(self):
        return self._finger_widgets


class GenericTabLayout(QWidget):
    def __init__(self, parent):
        super().__init__(parent=parent)

        self._subscriber = None
        self._CONST_FINGERS = ["ff", 'mf', 'rf', 'lf', 'th']
        self._finger_widgets = dict()
        self._timer = QTimer(self)

    def _init_tactile_layout(self):
        raise NotImplementedError("The function _init_tactile_layout must be implemented")

    def _tactile_data_callback(self):
        raise NotImplementedError("The function _tactile_data_callback must be implemented")

    def _initialize_data_structure(self):
        raise NotImplementedError("The function _initialize_data_structure must be implemented")

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()

    def get_finger_widgets(self):
        return self._finger_widgets


class GenericOptionBar(QGroupBox):
    def __init__(self, hand_ids, childs):
        super().__init__()
        self._hand_ids = hand_ids
        self._childs = childs
        self._CONST_FINGERS = ["ff", 'mf', 'rf', 'lf', 'th']
        self.setTitle("Options")
        self.setSizePolicy(1, 2)

    def init_layout(self):
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
        self.finger_selection_reset_button = QPushButton("Reset")
        self.finger_selection_reset_button.setSizePolicy(2, 2)

    def create_connections(self):
        self.hand_id_selection.currentIndexChanged.connect(self._combobox_action_hand_id_selection)
        self.finger_selection_show_selected_button.clicked.connect(self._button_action_show_selected_fingers)
        self.finger_selection_show_all_button.clicked.connect(self._button_action_show_all)
        self.finger_selection_reset_button.clicked.connect(self._button_action_reset)

    def _combobox_action_hand_id_selection(self):
        self._current_widget = self._childs.currentWidget()
        self._childs.setCurrentIndex(self.hand_id_selection.currentIndex())

    def _button_action_show_selected_fingers(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        self._selected_fingers = [finger for finger in self._CONST_FINGERS if fingertip_widgets[finger].isChecked()]
        for finger in self._CONST_FINGERS:
            if finger in self._selected_fingers:
                fingertip_widgets[finger].start_timer_and_subscriber()
                fingertip_widgets[finger].show()
            else:
                fingertip_widgets[finger].stop_timer_and_subscriber()
                fingertip_widgets[finger].hide()

    def _button_action_show_all(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        for finger in self._CONST_FINGERS:
            fingertip_widgets[finger].setChecked(True)
            fingertip_widgets[finger].start_timer_and_subscriber()
            fingertip_widgets[finger].show()

    def _button_action_reset(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        for finger in self._CONST_FINGERS:
            fingertip_widgets[finger].setChecked(False)
            fingertip_widgets[finger].stop_timer_and_subscriber()
            fingertip_widgets[finger].show()

    def _start_selected_widget(self, selected_widget):
        for i in range(self._childs.count()):
            finger_widgets_from_tab = self._childs.currentWidget().get_finger_widgets()
            for widget in finger_widgets_from_tab.values():
                if self._childs.currentWidget() == selected_widget:
                    widget.start_timer_and_subscriber()
                else:
                    widget.start_timer_and_subscriber()


class BiotacType(Enum):
    SP_PLUS = 0
    SP_MINUS = 1
    BLANK = 3

    @staticmethod
    def detect_biotac_type(side, selected_finger):
        sum_of_electrode_values = 0
        sp_plus_electrode_count_range = [1, 30]

        fingers = ['ff', 'mf', 'rf', 'lf', 'th']
        msg = rospy.wait_for_message("/{}/tactile".format(side), BiotacAll, timeout=1)
        for i, finger in enumerate(fingers):
            if finger == selected_finger:
                sum_of_electrode_values = sum(msg.tactiles[i].electrodes)

        if sum_of_electrode_values == 0:
            biotac_type = BiotacType.BLANK
        elif sp_plus_electrode_count_range[0] <= sum_of_electrode_values <= sp_plus_electrode_count_range[1]:
            biotac_type = BiotacType.SP_MINUS
        else:
            biotac_type = BiotacType.SP_PLUS
        return biotac_type
