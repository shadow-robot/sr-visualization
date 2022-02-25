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

from python_qt_binding.QtGui import QIcon, QColor,QPalette
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
    QComboBox
)

from sr_gui_fingertip_visualization.tab_data import GenericTabData
from sr_robot_msgs.msg import ShadowPST, BiotacAll

from sr_utilities.hand_finder import HandFinder
from sr_hand.tactile_receiver import TactileReceiver

class GenericTabLayout(QWidget, GenericTabData):
    def __init__(self, tab_name, parent=None):
        super().__init__(parent=parent)
        
        self._hand_ids = list(id.strip('_') for id in HandFinder().get_hand_parameters().joint_prefix.values())
        self._tactile_type = dict()

        for id in self._hand_ids:
            self._tactile_type[id] = TactileReceiver(id).find_tactile_type()
        

        # autodetection to be done later
        self._fingers = ["ff","mf","rf","lf","th"]
        self.finger_widgets = dict()

        self.initialize_data_structure()
        self.init_generic_layout()
        self.init_tactile_layout()           
        self.initialize_subscriber('rh')
        self.setLayout(self.main_tab_layout)
        self.initialize_and_start_timer()
        
    def init_generic_layout(self):
        self.main_tab_layout = QVBoxLayout()
        self.main_tab_layout.setContentsMargins(0, 0, 0, 0)
        
        # hand_id selection box
        self.hand_id_selection_layout = QFormLayout()
        self.hand_id_selection = QComboBox()                
        self.hand_id_selection.addItems(self._hand_ids)
        self.hand_id_selection_layout.addRow(QLabel("Hand ID:"), self.hand_id_selection)

        self.main_tab_layout.addLayout(self.hand_id_selection_layout)
    
    def initialize_and_start_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start()

    def init_tactile_layout(self):
        raise NotImplementedError("The function create_tab_options must be implemented")

    def timerEvent(self):
        raise NotImplementedError("The function create_tab_options must be implemented")

    def initialize_subscribers(self):
        raise NotImplementedError("The function get_data must be implemented, with a dictionary in return")

    def _tactile_data_callback(self):
        raise NotImplementedError("The function get_data must be implemented, with a dictionary in return")

    def initialize_data_structure(self):
        raise NotImplementedError("The function get_data must be implemented, with a dictionary in return")

class PSTVisualizationTab(GenericTabLayout):
    def __init__(self, tab_name, parent=None):
        super().__init__(tab_name, parent)

    def initialize_data_structure(self):
        self._data_fields = ['pressure', 'temperature']
        self._data_representation_types = ["_visual", "_text"]
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = 0
       
    def init_tactile_layout(self):
        fingers_frame = QHBoxLayout()
        for finger in self._fingers:
            fingers_frame.addWidget(self.create_finger_widget(finger))    
        self.main_tab_layout.addLayout(fingers_frame)

    def initialize_subscriber(self, side):
        self.sub = rospy.Subscriber('/{}/tactile'.format(side), ShadowPST, self._tactile_data_callback)

    def stop_subscribers(self, side):
        self.sub.unregister()

    def _tactile_data_callback(self, data):
        for data_field in self._data_fields:
            for i, finger in enumerate(self._fingers):         
                if data_field == "pressure":
                    self._data[finger][data_field] = data.pressure[i]
                elif data_field == "temperature":
                    self._data[finger][data_field] = data.temperature[i]

    def create_finger_widget(self, finger):               
        finger_layout = QFormLayout()
        self.finger_widgets[finger] = dict()

        for data_field in self._data_fields:
            for representation_type in self._data_representation_types:       
                self.finger_widgets[finger][data_field+representation_type] = None
                if representation_type == "_text":
                    self.finger_widgets[finger][data_field+representation_type] = QLineEdit()             
                elif representation_type == "_visual" and data_field != "temperature":
                    self.finger_widgets[finger][data_field+representation_type] = QProgressBar()    
                    self.finger_widgets[finger][data_field+representation_type].setRange(300,1000)
                    self.finger_widgets[finger][data_field+representation_type].setOrientation(Qt.Vertical)
                    self.finger_widgets[finger][data_field+representation_type].setAlignment(Qt.AlignCenter)

                if self.finger_widgets[finger][data_field+representation_type]:
                    finger_layout.addRow(QLabel(data_field+representation_type), self.finger_widgets[finger].get(data_field+representation_type)) 

        finger_frame = QGroupBox(finger)
        finger_frame.setAlignment(Qt.AlignHCenter)
        finger_frame.setLayout(finger_layout)

        return finger_frame

    def color_string(self, r, g, b):
        return "QLineEdit"+"{background : "+"rgb({}, {}, {});".format(r,g,b)+";}"
    
    def ranged_value_to_rgb(in_min, in_max, out_min, out_max, value):
        new_value = (value-in_min)*(out_max-out_min)/(in_max-in_min)+out_min
        
        green = 2*1
    
    def timerEvent(self):     

        for finger in self._fingers:
            self.finger_widgets[finger]['pressure_visual'].setValue(int(self._data[finger]["pressure"]))
            self.finger_widgets[finger]['pressure_text'].setText(str(self._data[finger]["pressure"]))
            #self.finger_widgets[finger]['temperature_visual'].setStyleSheet(self.color_string(0,100,0))
            self.finger_widgets[finger]['temperature_text'].setText(str(self._data[finger]["temperature"]))


    
