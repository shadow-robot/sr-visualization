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
        super().__init__(parent)

        self._side = side
        self._buffer_size = 100
        self._fingers = ['ff', 'mf', 'rf', 'lf', 'th']        
        self._data = dict()
        self._timer = QTimer()
        self._finger_frame = dict()
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
        for i, data_field in enumerate(self._data_fields):
            self._legend_colors[data_field] = dict()
            self._legend_colors[data_field]['icon'] = self.ICONS[available_colors[i]]
            self._legend_colors[data_field]['plot_color'] = QColor(available_colors[i])

        self._init_graph_layout()
        self.start_timer_and_subscriber()     

    def _init_graph_layout(self):
        finger_complete_layout = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            self._finger_frame[finger] = QGroupBox(finger)
            self._finger_frame[finger].setCheckable(True)
            self._finger_frame[finger].setSizePolicy(1, 1)
            
            self._data_selection[finger] = QHBoxLayout()
            self._data_selection_checkboxes[finger] = dict()

            finger_complete_layout.addWidget(self._init_finger_widget(finger))

        self.setLayout(finger_complete_layout)       

    def _initialize_data_structure(self):
        raise NotImplementedError("The function _initialize_data_structure must be implemented")
    def start_timer_and_subscriber(self):
        raise NotImplementedError("The function start_timer_and_subscriber must be implemented")


class PSTGraphTab(GenericGraphTab):
    def __init__(self, side, parent):
        super().__init__(side, parent)

    def _initialize_data_structure(self):
        self._data_fields = ['pressure', 'temperature']
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = list()

    def _init_finger_widget(self, finger):

        for data_field in self._data_fields:
            self._data_selection_checkboxes[finger][data_field] = QCheckBox(data_field)
            self._data_selection_checkboxes[finger][data_field].setIcon(self._legend_colors[data_field]['icon'])
            self._data_selection[finger].addWidget(self._data_selection_checkboxes[finger][data_field])

        finger_layout = QVBoxLayout() 
        finger_layout.addLayout(self._data_selection[finger])

        self._plot[finger] = GenericDataPlot(self._data[finger], self._legend_colors)
        self._plot[finger].setMinimumSize(50, 50)
        finger_layout.addWidget(self._plot[finger])
        
        self._finger_frame[finger].setLayout(finger_layout)

        return self._finger_frame[finger]

    def start_timer_and_subscriber(self):
        self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), ShadowPST, self._tactile_data_callback)
        self._timer.timeout.connect(self.timerEvent)
        self._timer.start(10)

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            for data_field in self._data_fields:
                if len(self._data[finger][data_field]) >= self._buffer_size:
                    self._data[finger][data_field] = self._data[finger][data_field][1:]           
                if data_field == "pressure":
                    self._data[finger][data_field].append(data.pressure[i]) 
                elif data_field == "temperature":
                    self._data[finger][data_field].append(data.temperature[i])    

    def timerEvent(self):
        for finger in self._fingers:
            traces_to_show = list()
            for data_field in self._data_fields:                
                if self._data_selection_checkboxes[finger][data_field].isChecked():
                    self._plot[finger].update_plot(self._data[finger])
                    traces_to_show.append(data_field)
            self._plot[finger].show_traces(traces_to_show)


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
        self._plot[finger].setMinimumSize(50, 50)
        finger_layout.addWidget(self._plot[finger])
        
        self._finger_frame[finger].setLayout(finger_layout)
        return self._finger_frame[finger]

    def start_timer_and_subscriber(self):
        self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), BiotacAll, self._tactile_data_callback)
        self._timer.timeout.connect(self.timerEvent)
        self._timer.start(10)

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()

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

    def timerEvent(self):
        for finger in self._fingers:
            traces_to_show = list()
            for data_field in self._data_fields:                
                if self._data_selection_checkboxes[finger][data_field].isChecked():
                    self._plot[finger].update_plot(self._data[finger])
                    traces_to_show.append(data_field)
            self._plot[finger].show_traces(traces_to_show)    

class GraphTab(QWidget):
    def __init__(self, tactile_topics):
        super().__init__()
        self._tactile_topics = tactile_topics
        self._init_layout()
        self._create_connections()

    def _init_layout(self):

        finger_layout = QVBoxLayout(self)

        options_layout_groupbbox = QGroupBox("Options")
        options_layout_groupbbox.setSizePolicy(1, 2)
        options_layout = QHBoxLayout()

        hand_id_selection_layout = QFormLayout()
        self.hand_id_selection = QComboBox()
        self.hand_id_selection.addItems(self._tactile_topics.keys())
        hand_id_selection_layout.addRow(QLabel("Hand ID:"), self.hand_id_selection)

        finger_selection_label = QLabel("Finger selection:")
        self.finger_selection_show_selected_button = QPushButton("Show selected")
        self.finger_selection_show_selected_button.setSizePolicy(2, 2)
        self.finger_selection_show_all_button = QPushButton("Show all")
        self.finger_selection_show_all_button.setSizePolicy(2, 2)

        options_layout.addLayout(hand_id_selection_layout)
        options_layout.addStretch(1)
        options_layout.addWidget(finger_selection_label)
        options_layout.addWidget(self.finger_selection_show_selected_button)
        options_layout.addWidget(self.finger_selection_show_all_button)

        options_layout_groupbbox.setLayout(options_layout)
        self.stacked_layout = QStackedLayout(self)

        self.fingertip_widget = dict()
        for side, tactile_topic in self._tactile_topics.items():
            if tactile_topic == "ShadowPST":
                self.fingertip_widget[side] = PSTGraphTab(side, self)
            elif tactile_topic == "BiotacAll":
                self.fingertip_widget[side] = BiotacGraphTab(side, self)
            self.stacked_layout.addWidget(self.fingertip_widget[side])

        finger_layout.addWidget(options_layout_groupbbox)
        finger_layout.addLayout(self.stacked_layout)
        self.setLayout(finger_layout)

        self._current_side = list(self._tactile_topics.keys())[0]
        #self._start_selected_widget(self.fingertip_widget[self._current_side])

    def _create_connections(self):
        self.hand_id_selection.currentIndexChanged.connect(self._combobox_action_hand_id_selection)

    def _combobox_action_hand_id_selection(self):
        self._current_side = self.hand_id_selection.currentText()
        self.stacked_layout.setCurrentWidget(self.fingertip_widget[self._current_side])
        self._start_selected_widget(self.fingertip_widget[self._current_side])

    def _start_selected_widget(self, selected_widget):
        for widget in self.fingertip_widget.values():
            if widget == selected_widget:
                widget.start_timer_and_subscriber()
            else:
                widget.stop_timer_and_subscriber()
