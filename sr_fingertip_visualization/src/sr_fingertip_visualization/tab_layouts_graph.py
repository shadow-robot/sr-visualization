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
from python_qt_binding.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QStackedLayout
)
from sr_fingertip_visualization.tab_layouts_generic import (
    GenericGraphTab,
    GenericOptionBar,
    BiotacType
)
from sr_fingertip_visualization.finger_widgets_graphs import (
    FingerWidgetGraphPST,
    FingerWidgetGraphBiotac,
    FingerWidgetGraphBiotacBlank
)


class PSTGraphTab(GenericGraphTab):

    CONST_DATA_FIELDS = ['pressure', 'temperature']

    def __init__(self, side, parent):
        super().__init__(side, parent)
        self._side = side
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        for finger in self._CONST_FINGERS:
            self._data[finger] = {}
            for data_field in self.CONST_DATA_FIELDS:
                self._data[finger][data_field] = []

    def _init_tactile_layout(self):
        fingers_layout = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            fingers_layout.addWidget(self._init_finger_widget(finger))
        self.setLayout(fingers_layout)

    def _init_finger_widget(self, finger):
        self._finger_widgets[finger] = FingerWidgetGraphPST(self._side, finger, self)
        return self._finger_widgets[finger]


class BiotacGraphTab(GenericGraphTab): # pylint: disable=W0223

    CONST_DATA_FIELDS = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']

    def __init__(self, side, parent):
        super().__init__(side, parent)
        self._side = side
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        for finger in self._CONST_FINGERS:
            self._data[finger] = {}
            for data_field in self.CONST_DATA_FIELDS:
                self._data[finger][data_field] = []

    def _init_tactile_layout(self):
        fingers_layout = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            fingers_layout.addWidget(self._init_finger_widget(finger))
        self.setLayout(fingers_layout)

    def _init_finger_widget(self, finger):
        if BiotacType.detect_biotac_type(self._side, finger) == BiotacType.BLANK:
            self._finger_widgets[finger] = FingerWidgetGraphBiotacBlank(self._side, finger, self)
        else:
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

        self.tactile_widgets = {}
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
        self.options_layout.addWidget(self.finger_selection_reset_button)

        self.setLayout(self.options_layout)
        self._current_widget = self._childs.currentWidget()

    def _button_action_show_all(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        self._selected_fingers = [finger for finger in self._CONST_FINGERS if fingertip_widgets[finger].isChecked()]
        for finger in self._CONST_FINGERS:
            fingertip_widgets[finger].setChecked(True)
            # pylint thinks the functions empty so have to disable check
            fingertip_widgets[finger].start_timer_and_subscriber() # pylint: disable=W0223
            fingertip_widgets[finger].show()

            for data_checkbox in fingertip_widgets[finger].get_data_checkboxes().values():
                data_checkbox.setChecked(True)

    def _button_action_reset(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        for finger in self._CONST_FINGERS:
            fingertip_widgets[finger].setChecked(False)
            fingertip_widgets[finger].stop_timer_and_subscriber()
            fingertip_widgets[finger].show()

            for data_checkbox in fingertip_widgets[finger].get_data_checkboxes().values():
                data_checkbox.setChecked(False)
