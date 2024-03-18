#!/usr/bin/env python3

# Copyright 2022, 2024 Shadow Robot Company Ltd.
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

import rospy
from sr_robot_msgs.msg import BiotacAll
from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QStackedLayout
)

from sr_fingertip_visualization.tab_layouts_generic import (
    GenericTabLayout,
    GenericOptionBar,
    BiotacType
)
from sr_fingertip_visualization.finger_widgets_visual import (
    FingerWidgetVisualBiotacSPMinus,
    FingerWidgetVisualBiotacSPPlus,
    FingerWidgetVisualBiotacBlank,
    FingerWidgetVisualPST,
    FingerWidgetVisualMSTBlank
)


class VisualizationTab(QWidget):

    _CONST_FINGERS = ['th', 'ff', 'mf', 'rf', 'lf']

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
                self.tactile_widgets[side] = PSTVisualizationTab(side, parent=self)
            elif tactile_topic == "BiotacAll":
                self.tactile_widgets[side] = BiotacVisualizationTab(side, parent=self)
            elif tactile_topic == "MSTAll":
                rospy.logwarn(f"Found STF sensors on {side} hand, but no visualization is officially supported yet.")
                self.tactile_widgets[side] = MSTVisualizationTab(side, parent=self)
            self.stacked_layout.addWidget(self.tactile_widgets[side])
        self._option_bar = VisualOptionBar(list(self._tactile_topics.keys()), childs=self.stacked_layout)

        finger_layout.addWidget(self._option_bar)
        finger_layout.addLayout(self.stacked_layout)
        self.setLayout(finger_layout)

    def get_tactile_widgets(self):
        return self.tactile_widgets


class PSTVisualizationTab(GenericTabLayout):  # pylint: disable=W0223

    CONST_DATA_FIELDS = ['pressure', 'temperature']

    def __init__(self, side, parent):
        super().__init__(parent=parent)
        self._side = side
        self._initialize_data_structure()
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        self._data = dict.fromkeys(self._CONST_FINGERS, dict.fromkeys(self.CONST_DATA_FIELDS, 0))

    def _tactile_data_callback(self, data):
        # pylint: disable=W0221
        for i, finger in enumerate(self._CONST_FINGERS):
            for data_field in self.CONST_DATA_FIELDS:
                if data_field == "pressure":
                    self._data[finger][data_field] = data.pressure[i]
                elif data_field == "temperature":
                    self._data[finger][data_field] = data.temperature[i]

    def _init_tactile_layout(self):
        fingers_layout = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            fingers_layout.addWidget(self._init_finger_widget(finger))
        self.setLayout(fingers_layout)

    def _init_finger_widget(self, finger):
        self._finger_widgets[finger] = FingerWidgetVisualPST(self._side, finger, self)
        return self._finger_widgets[finger]


class BiotacVisualizationTab(GenericTabLayout):

    CONST_DATA_FIELDS = ['pac0', 'pac1', 'pac', 'pdc', 'tac', 'tdc', 'electrodes']

    def __init__(self, side, parent):
        super().__init__(parent=parent)
        self._side = side
        self._electrode_count = 0
        self._version = "v1"
        self._initialize_data_structure()
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        self._data = {}
        self._data_labels = {}
        for finger in self._CONST_FINGERS:
            self._data[finger] = {}
            self._data_labels[finger] = {}
            for data_field in self.CONST_DATA_FIELDS:
                self._data[finger][data_field] = 0
                self._data_labels[finger] = {}
                if data_field in ['pac', 'electrodes']:
                    self._data[finger][data_field] = []
                    self._data_labels[finger][data_field] = 0

        msg = rospy.wait_for_message('/{}/tactile'.format(self._side), BiotacAll)
        self._version = "v2" if len(msg.tactiles[0].electrodes) == 24 else "v1"
        self._tactile_data_callback(msg)

        self._coordinates = {}
        self._coordinates['v1'] = {}
        self._coordinates['v1']['sensing'] = {}
        self._coordinates['v1']['sensing']['x'] = [6.45, 3.65, 3.65, 6.45, 3.65, 6.45, 0.00, 1.95, -1.95, 0.00,
                                                   -6.45, - 3.65, -3.65, -6.45, -3.65, -6.45, 0.00, 0.00, 0.00]
        self._coordinates['v1']['sensing']['y'] = [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38, 6.38, 8.38,
                                                   7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38, 18.38, 22.18]
        self._coordinates['v1']['excitation'] = {}
        self._coordinates['v1']['excitation']['x'] = [6.45, 3.75, -3.75, -6.45]
        self._coordinates['v1']['excitation']['y'] = [12.48, 24.48, 24.48, 12.48]

        self._coordinates['v2'] = {}
        self._coordinates['v2']['sensing'] = {}
        self._coordinates['v2']['sensing']['x'] = [5.00, 3.65, 6.45, 4.40, 2.70, 6.45, 4.40, 1.50, 4.00, 4.50, -5.00,
                                                   -3.65, -6.45, -4.40, -2.70, -6.45, -4.40, -1.50, -4.00, -4.50, 0.00,
                                                   1.95, -1.95, 0.00]
        self._coordinates['v2']['sensing']['y'] = [4.38, 6.38, 10.78, 11.50, 14.50, 15.08, 16.00, 17.00, 19.00, 21.00,
                                                   4.38, 6.38, 10.78, 11.50, 14.50, 15.08, 16.00, 17.00, 19.00, 21.00,
                                                   7.38, 9.50, 9.50, 11.20]
        self._coordinates['v2']['excitation'] = {}
        self._coordinates['v2']['excitation']['x'] = [5.30, 6.00, -5.30, -6.00]
        self._coordinates['v2']['excitation']['y'] = [9.00, 22.00, 9.00, 22.00]

        self._electrode_count = len(self._coordinates[self._version]['sensing']['x'])

    def _tactile_data_callback(self, data):
        # pylint: disable=W0221
        for i, finger in enumerate(self._CONST_FINGERS):
            for data_field in self.CONST_DATA_FIELDS:
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

    def _init_tactile_layout(self):
        finger_complete_layout = QHBoxLayout()
        self.text_data_layout = {}
        self._data_bar = {}
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            finger_complete_layout.addWidget(self._init_finger_widget(finger))
        self.setLayout(finger_complete_layout)

    def _init_finger_widget(self, finger):
        detected_fingertip_type = BiotacType.detect_biotac_type(self._side, finger)
        if detected_fingertip_type == BiotacType.SP_PLUS:
            self._finger_widgets[finger] = FingerWidgetVisualBiotacSPPlus(self._side, finger, self)
        elif detected_fingertip_type == BiotacType.SP_MINUS:
            self._finger_widgets[finger] = FingerWidgetVisualBiotacSPMinus(self._side, finger, self)
        elif detected_fingertip_type == BiotacType.BLANK:
            self._finger_widgets[finger] = FingerWidgetVisualBiotacBlank(finger, self)
        return self._finger_widgets[finger]


class MSTVisualizationTab(GenericTabLayout):  # pylint: disable=W0223

    def __init__(self, side, parent):
        super().__init__(parent=parent)
        self._side = side
        self._init_tactile_layout()
        self._finger_widget = None

    def _init_tactile_layout(self):
        fingers_layout = QHBoxLayout()
        fingers_layout.setContentsMargins(0, 0, 0, 0)
        fingers_layout.addWidget(self._init_finger_widget())
        self.setLayout(fingers_layout)

    def _init_finger_widget(self):
        self._finger_widget = FingerWidgetVisualMSTBlank(self._side, self)
        return self._finger_widget


class VisualOptionBar(GenericOptionBar):
    def __init__(self, hand_ids, childs):
        super().__init__(hand_ids, childs)
        self.init_layout()
        self.create_connections()
        self._current_widget = None

    def init_layout(self):
        super().init_layout()
        self.data_type_selection_button = QPushButton("Show pac")
        self.options_layout.addLayout(self.hand_id_selection_layout)
        self.options_layout.addStretch(1)
        self.options_layout.addWidget(self.data_type_selection_button)
        self.options_layout.addWidget(self.finger_selection_label)
        self.options_layout.addWidget(self.finger_selection_show_selected_button)
        self.options_layout.addWidget(self.finger_selection_show_all_button)
        self.options_layout.addWidget(self.finger_selection_reset_button)

        self.setLayout(self.options_layout)
        self._current_widget = self._childs.currentWidget()

    def create_connections(self):
        super().create_connections()
        self.data_type_selection_button.clicked.connect(self._button_action_data_type_selection)

    def _combobox_action_hand_id_selection(self):
        self._current_widget = self._childs.currentWidget()
        self._childs.setCurrentIndex(self.hand_id_selection.currentIndex())

    def _button_action_data_type_selection(self):
        data_type_options_to_display = ['pac', 'electrodes']

        for widget in self._childs.currentWidget().get_finger_widgets().values():
            if isinstance(widget, FingerWidgetVisualBiotacSPPlus):
                opposite_option = [option for option in data_type_options_to_display
                                   if option is not widget.get_datatype_to_display()][0]
                widget.change_datatype_to_display(opposite_option)
                self.data_type_selection_button.setText("Show {}".format(opposite_option))

    def _button_action_show_all(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        for finger in self._CONST_FINGERS:
            fingertip_widgets[finger].setChecked(True)
            fingertip_widgets[finger].start_timer_and_subscriber()
            fingertip_widgets[finger].show()
