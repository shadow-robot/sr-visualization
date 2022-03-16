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
import rospy
from enum import Enum

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QFormLayout,
    QLabel,
    QComboBox,
    QStackedLayout
)

from sr_gui_fingertip_visualization.tactile_points import TactilePointPST, TactilePointBiotacSPPlus, TactilePointBiotacSPMinus
from sr_gui_fingertip_visualization.tab_layouts_generic import GenericTabLayout
from sr_gui_fingertip_visualization.finger_widgets_visual import (
    FingerWidgetVisualBiotacSPMinus, 
    FingerWidgetVisualBiotacSPPlus, 
    FingerWidgetVisualBiotacBlank, 
    FingerWidgetVisualPST
)
from sr_robot_msgs.msg import ShadowPST, BiotacAll


class VisualizationTab(QWidget):
    def __init__(self, tactile_topics):
        super().__init__()
        self._tactile_topics = tactile_topics
        self._init_layout()
        self._fingers = ['th', 'ff', 'mf', 'rf', 'lf']

    def _init_layout(self):
        finger_layout = QVBoxLayout()    
        self.stacked_layout = QStackedLayout()
        self.tactile_widgets = dict()
        for side, tactile_topic in self._tactile_topics.items():
            if tactile_topic == "ShadowPST":
                self.tactile_widgets[side] = PSTVisualizationTab(side, parent=self)
            elif tactile_topic == "BiotacAll":
                self.tactile_widgets[side] = BiotacVisualizationTab(side, parent=self)
            self.stacked_layout.addWidget(self.tactile_widgets[side])
        self._option_bar = OptionBar(list(self._tactile_topics.keys()), childs=self.stacked_layout)
        
        finger_layout.addWidget(self._option_bar)
        finger_layout.addLayout(self.stacked_layout)        
        self.setLayout(finger_layout)

    def get_tactile_widgets(self):
        return self.tactile_widgets

class PSTVisualizationTab(GenericTabLayout):
    def __init__(self, side, parent):
        super().__init__(parent=parent)
        self._side = side
        self._initialize_data_structure()
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        self._data_fields = ['pressure', 'temperature']
        self._data = dict.fromkeys(self._fingers, dict.fromkeys(self._data_fields, 0))

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            for data_field in self._data_fields:
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
    def __init__(self, side, parent):
        super().__init__(parent=parent)
        self._side = side        
        self._electrode_count = 0
        self._version = "v1"
        self._selected_fingers = []
        self._initialize_data_structure()
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        self._data_fields = ['pac0', 'pac1', 'pac', 'pdc', 'tac', 'tdc', 'electrodes']
        self._data = dict()
        self._data_labels = dict()
        for finger in self._fingers:
            self._data[finger] = dict()
            self._data_labels[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = 0
                self._data_labels[finger] = dict()
                if data_field in ['pac', 'electrodes']:
                    self._data[finger][data_field] = list()   
                    self._data_labels[finger][data_field] = 0

        msg = rospy.wait_for_message('/{}/tactile'.format(self._side), BiotacAll)
        self._version = "v2" if len(msg.tactiles[0].electrodes) == 24 else "v1"
        self._tactile_data_callback(msg)

        self._coordinates = dict()
        self._coordinates['v1'] = dict()
        self._coordinates['v1']['sensing'] = dict()
        self._coordinates['v1']['sensing']['x'] = [6.45, 3.65, 3.65, 6.45, 3.65, 6.45, 0.00, 1.95, -1.95, 0.00,
                                                   -6.45, - 3.65, -3.65, -6.45, -3.65, -6.45, 0.00, 0.00, 0.00]
        self._coordinates['v1']['sensing']['y'] = [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38, 6.38, 8.38,
                                                   7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38, 18.38, 22.18]
        self._coordinates['v1']['excitation'] = dict()
        self._coordinates['v1']['excitation']['x'] = [6.45, 3.75, -3.75, -6.45]
        self._coordinates['v1']['excitation']['y'] = [12.48, 24.48, 24.48, 12.48]

        self._coordinates['v2'] = dict()
        self._coordinates['v2']['sensing'] = dict()
        self._coordinates['v2']['sensing']['x'] = [5.00, 3.65, 6.45, 4.40, 2.70, 6.45, 4.40, 1.50, 4.00, 4.50, -5.00,
                                                   -3.65, -6.45, -4.40, -2.70, -6.45, -4.40, -1.50, -4.00, -4.50, 0.00,
                                                   1.95, -1.95, 0.00]
        self._coordinates['v2']['sensing']['y'] = [4.38, 6.38, 10.78, 11.50, 14.50, 15.08, 16.00, 17.00, 19.00, 21.00,
                                                   4.38, 6.38, 10.78, 11.50, 14.50, 15.08, 16.00, 17.00, 19.00, 21.00,
                                                   7.38, 9.50, 9.50, 11.20]
        self._coordinates['v2']['excitation'] = dict()
        self._coordinates['v2']['excitation']['x'] = [5.30, 6.00, -5.30, -6.00]
        self._coordinates['v2']['excitation']['y'] = [9.00, 22.00, 9.00, 22.00]
        
        self._electrode_count = len(self._coordinates[self._version]['sensing']['x'])

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            for data_field in self._data_fields:            
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
        self.text_data_layout = dict()
        self._data_bar = dict()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:            
            finger_complete_layout.addWidget(self._init_finger_widget(finger))
        self.setLayout(finger_complete_layout)

    def _init_finger_widget(self, finger):        
        detected_fingertip_type = self._detect_biotac_type(finger)
        if detected_fingertip_type == BiotacType.SP_PLUS:
            self._finger_widgets[finger] = FingerWidgetVisualBiotacSPPlus(self._side, finger, self)
        elif detected_fingertip_type == BiotacType.SP_MINUS:
            self._finger_widgets[finger] = FingerWidgetVisualBiotacSPMinus(self._side, finger, self)
        elif detected_fingertip_type == BiotacType.BLANK:
            self._finger_widgets[finger] = FingerWidgetVisualBiotacBlank(finger, self)
        return self._finger_widgets[finger]

    def _detect_biotac_type(self, finger):
        sum_of_electrode_values = sum(self._data[finger]['electrodes'])
        if sum_of_electrode_values == 0:
            biotac_type = BiotacType.BLANK
        elif sum_of_electrode_values == self._electrode_count:
            biotac_type = BiotacType.SP_MINUS
        else:
            biotac_type = BiotacType.SP_PLUS
        return biotac_type


class BiotacType(Enum):
    SP_PLUS = 0
    SP_MINUS = 1
    BLANK = 3


class BiotacSPPlusInfo(QGroupBox):
    def __init__(self, parent):
        super().__init__(parent=parent)
        self._text_fields = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']
        self._data = dict.fromkeys(self._text_fields, 0)
        self._labels = dict()

        self.setSizePolicy(1,2)
        self.setTitle("Data")

        layout_pressure = QHBoxLayout()
        layout_temperature = QHBoxLayout()

        for key in self._text_fields:
            self._labels[key] = QLabel(self)
            self._labels[key].setText(f"{key}:0")
            if key[0] == 'p':
                layout_pressure.addWidget(self._labels[key])
            elif key[0] == 't':
                layout_temperature.addWidget(self._labels[key])
        
        layout = QVBoxLayout()
        layout.addLayout(layout_pressure)
        layout.addLayout(layout_temperature)
        self.setLayout(layout)

    def update_values(self, data):
        common_keys = list(set(data.keys()) & set(self._text_fields))
        for key in common_keys:
            self._data[key] = data[key]

    def get_widget(self):
        return widget
        
    def refresh(self):
        for key in self._text_fields:
            self._labels[key].setText(f"{key}:{self._data[key]}") 


class OptionBar(QGroupBox):
    def __init__(self, hand_ids, childs):
        super().__init__()
        self._childs = childs
        self._fingers = ["ff", 'mf', 'rf', 'lf', 'th']

        self.setTitle("Options")
        self.setSizePolicy(1, 2)

        options_layout = QHBoxLayout()

        hand_id_selection_layout = QFormLayout()
        self.hand_id_selection = QComboBox()
        self.hand_id_selection.addItems(hand_ids)
        hand_id_selection_layout.addRow(QLabel("Hand ID:"), self.hand_id_selection)

        self.data_type_selection_button = QPushButton("Show pac")

        finger_selection_label = QLabel("Finger selection:")
        self.finger_selection_show_selected_button = QPushButton("Show selected")
        self.finger_selection_show_selected_button.setSizePolicy(2, 2)
        self.finger_selection_show_all_button = QPushButton("Show all")
        self.finger_selection_show_all_button.setSizePolicy(2, 2)

        options_layout.addLayout(hand_id_selection_layout)
        options_layout.addStretch(1)
        options_layout.addWidget(self.data_type_selection_button)
        options_layout.addWidget(finger_selection_label)
        options_layout.addWidget(self.finger_selection_show_selected_button)
        options_layout.addWidget(self.finger_selection_show_all_button)

        self.setLayout(options_layout)
        self._current_widget = self._childs.currentWidget()        
        self._create_connections()

    def _create_connections(self):
        self.hand_id_selection.currentIndexChanged.connect(self._combobox_action_hand_id_selection)
        self.data_type_selection_button.clicked.connect(self._button_action_data_type_selection)
        self.finger_selection_show_selected_button.clicked.connect(self._button_action_show_selected_fingers)
        self.finger_selection_show_all_button.clicked.connect(self._button_action_show_all)

    def _combobox_action_hand_id_selection(self):
        self._current_widget = self._childs.currentWidget()        
        self._childs.setCurrentIndex(self.hand_id_selection.currentIndex())

    def _button_action_data_type_selection(self):
        data_type_options_to_display = ['pac', 'electrodes']

        for finger, widget in self._childs.currentWidget().get_finger_widgets().items():
            if isinstance(widget, FingerWidgetBiotacSPPlus):               
                opposite_option = [option for option in data_type_options_to_display if option is not widget.get_datatype_to_display()][0]
                widget.change_datatype_to_display(opposite_option)
                self.data_type_selection_button.setText("Show {}".format(opposite_option))

    def _button_action_show_selected_fingers(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        self._selected_fingers = [finger for finger in self._fingers if fingertip_widgets[finger].isChecked()]
        for finger in self._fingers:
            if finger in self._selected_fingers:
                fingertip_widgets[finger].show()
            else:
                fingertip_widgets[finger].hide()

    def _button_action_show_all(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_widgets()
        self._selected_fingers = [finger for finger in self._fingers if fingertip_widgets[finger].isChecked()]
        for finger in self._fingers:
            fingertip_widgets[finger].stop_timer_and_subscriber()
            fingertip_widgets[finger].setChecked(False)
            fingertip_widgets[finger].show()

    # to remove
    def _start_selected_widget(self, selected_widget):
        for widget_index in range(self._childs.count()):
            finger_widgets_from_tab = self._childs.currentWidget().get_finger_widgets()
            for finger, widget in finger_widgets_from_tab.items():
                if self._childs.currentWidget() == selected_widget:
                    widget.start_timer_and_subscriber()
                else:
                    widget.stop_timer_and_subscriber()
