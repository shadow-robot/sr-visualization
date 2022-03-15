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
from sr_gui_fingertip_visualization.tab_layouts_generic import GenericGraphTab


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


from sr_gui_fingertip_visualization.finger_widgets_graphs import FingerWidgetGraphPST


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
        rospy.logwarn("init")

    def _init_finger_widget(self, finger):
        self._finger_widgets[finger] = FingerWidgetGraphPST(self._side, finger, self)
        return self._finger_widgets[finger]

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            for data_field in self._data_fields:
                if len(self._data[finger][data_field]) >= self._buffer_size:
                    self._data[finger][data_field] = self._data[finger][data_field][1:]           
                if data_field == "pressure":
                    self._data[finger][data_field].append(data.pressure[i]) 
                elif data_field == "temperature":
                    self._data[finger][data_field].append(data.temperature[i])    


class BiotacGraphTab(GenericGraphTab):
    def __init__(self, side, parent):
        super().__init__(side, parent)
       
    def _initialize_data_structure(self):
        self._data_fields = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = list()

    def _init_finger_widget(self, finger):
        group_box_pressure = QGroupBox("Pressure", self)
        layout_pressure = QHBoxLayout()
        group_box_temperature = QGroupBox("Temperature", self)        
        layout_temperature = QHBoxLayout()
        
        for data_field in self._data_fields:
            self._data_selection_checkboxes[finger][data_field] = QCheckBox(data_field)
            self._data_selection_checkboxes[finger][data_field].setIcon(self._legend_colors[data_field]['icon'])
            if data_field in ['pac0', 'pac1', 'pdc']:
                layout_pressure.addWidget(self._data_selection_checkboxes[finger][data_field])
            elif data_field in ['tac', 'tdc']:  
                layout_temperature.addWidget(self._data_selection_checkboxes[finger][data_field])
        
        group_box_pressure.setLayout(layout_pressure)
        group_box_temperature.setLayout(layout_temperature)
    
        self._data_selection[finger].addWidget(group_box_pressure)
        self._data_selection[finger].addWidget(group_box_temperature)

        finger_layout = QVBoxLayout()
        finger_layout.addLayout(self._data_selection[finger])

        self._plot[finger] = GenericDataPlot(self._data[finger], self._legend_colors)
        finger_layout.addWidget(self._plot[finger])
        
        self._finger_frame[finger].setLayout(finger_layout)
        return self._finger_frame[finger]

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            for data_field in self._data_fields:
                if len(self._data[finger][data_field]) >= self._buffer_size:
                    self._data[finger][data_field] = self._data[finger][data_field][1:]           

                if data_field == "pac0":
                    self._data[finger][data_field].append(data.tactiles[i].pac0)
                elif data_field == "pac1":
                    self._data[finger][data_field].append(data.tactiles[i].pac1)
                elif data_field == "pdc":
                    self._data[finger][data_field].append(data.tactiles[i].pdc)
                elif data_field == "tac":
                    self._data[finger][data_field].append(data.tactiles[i].tac)
                elif data_field == "tdc":
                    self._data[finger][data_field].append(data.tactiles[i].tdc)


class GraphTab(QWidget):
    def __init__(self, tactile_topics):
        super().__init__()
        self._tactile_topics = tactile_topics
        self._init_layout()

    def _init_layout(self):
        finger_layout = QVBoxLayout()
        self.stacked_layout = QStackedLayout()

        self.fingertip_widget = dict()
        for side, tactile_topic in self._tactile_topics.items():
            if tactile_topic == "ShadowPST":
                self.fingertip_widget[side] = PSTGraphTab(side, self)
            elif tactile_topic == "BiotacAll":
                self.fingertip_widget[side] = BiotacGraphTab(side, self)
            self.stacked_layout.addWidget(self.fingertip_widget[side])

        self._option_bar = OptionBar(list(self._tactile_topics.keys()), childs=self.stacked_layout)
        
        finger_layout.addWidget(self._option_bar)        
        finger_layout.addLayout(self.stacked_layout)
        self.setLayout(finger_layout)

    def get_fingertip_widgets(self):
        return self.fingertip_widget


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
        self._start_selected_widget(self._current_widget)
        self._childs.setCurrentIndex(self.hand_id_selection.currentIndex())
        self._create_connections()

    def _create_connections(self):
        self.hand_id_selection.currentIndexChanged.connect(self._combobox_action_hand_id_selection)
        self.data_type_selection_button.clicked.connect(self._button_action_data_type_selection)
        self.finger_selection_show_selected_button.clicked.connect(self._button_action_show_selected_fingers)
        self.finger_selection_show_all_button.clicked.connect(self._button_action_show_all)

    def _combobox_action_hand_id_selection(self):
        self._current_widget = self._childs.currentWidget()        
        self._start_selected_widget(self._current_widget)
        self._childs.setCurrentIndex(self.hand_id_selection.currentIndex())

    def _button_action_data_type_selection(self):
        self._data_type_options_to_display = ["pac", "electrodes"]
        if isinstance(self._childs.currentWidget(), BiotacVisualizationTab):
            for datatype in self._data_type_options_to_display:
                if datatype in self.data_type_selection_button.text():
                    self._childs.currentWidget().change_datatype_to_display(datatype)
                    opposite_option = [i for i in self._data_type_options_to_display if i is not datatype][0]
                    self.data_type_selection_button.setText("Show {}".format(opposite_option))
                    break

    def _button_action_show_selected_fingers(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_frames()
        self._selected_fingers = [finger for finger in self._fingers if fingertip_widgets[finger].isChecked()]
        for finger in self._fingers:
            if finger in self._selected_fingers:
                fingertip_widgets[finger].show()
            else:
                fingertip_widgets[finger].hide()

    def _button_action_show_all(self):
        fingertip_widgets = self._childs.currentWidget().get_finger_frames()
        self._selected_fingers = [finger for finger in self._fingers if fingertip_widgets[finger].isChecked()]
        for finger in self._fingers:
            fingertip_widgets[finger].setChecked(False)
            fingertip_widgets[finger].show()

    def _start_selected_widget(self, selected_widget):
        for widget_index in range(self._childs.count()):
            finger_widgets_from_tab = self._childs.currentWidget().get_finger_widgets()
            rospy.logwarn(finger_widgets_from_tab)
            for finger, widget in finger_widgets_from_tab.items():
                if self._childs.currentWidget() == selected_widget:
                    widget.start_timer_and_subscriber()
                else:
                    widget.start_timer_and_subscriber()
