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

from python_qt_binding.QtGui import QIcon, QColor,QPalette, QFontMetrics
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
from sr_gui_fingertip_visualization.dot_unit import DotUnitPST
from sr_robot_msgs.msg import ShadowPST, BiotacAll

from sr_utilities.hand_finder import HandFinder
from sr_hand.tactile_receiver import TactileReceiver


class GenericTabLayout(QWidget, GenericTabData):
    def __init__(self, tab_name, painter):
        super().__init__()
        self.painter = painter
        self._hand_ids = list(id.strip('_') for id in HandFinder().get_hand_parameters().joint_prefix.values())
        self._tactile_type = dict()
        for id in self._hand_ids:
            self._tactile_type[id] = TactileReceiver(id).find_tactile_type()        

        self._fingers = ["ff",'mf','rf','lf','th']
        self.finger_widgets = dict()

        self.initialize_data_structure()
        self.init_generic_layout()
        self.init_tactile_layout()
        self.setLayout(self.main_tab_layout)        

        self.initialize_subscriber('rh')
        self._tactile_data_callback(rospy.wait_for_message('/rh/tactile', BiotacAll))
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
        self.timer.start(10)

    def init_tactile_layout(self):
        raise NotImplementedError("The function create_tab_options must be implemented")

    def initialize_subscribers(self):
        raise NotImplementedError("The function get_data must be implemented, with a dictionary in return")

    def _tactile_data_callback(self):
        raise NotImplementedError("The function get_data must be implemented, with a dictionary in return")

    def initialize_data_structure(self):
        raise NotImplementedError("The function get_data must be implemented, with a dictionary in return")


class PSTVisualizationTab(GenericTabLayout):
    def __init__(self, tab_name, parent):
        super().__init__(tab_name, parent)
        rospy.logwarn("PST tab")

    def initialize_data_structure(self):
        self._data_fields = ['pressure', 'temperature']
        self._data_representation_types = ["_visual", "_text"]
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = 0
       
    def init_tactile_layout(self):
        fingers_frame = QHBoxLayout()
        self.finger_widget = dict()
        for finger in self._fingers:
            self.finger_widget[finger] = DotUnitPST(finger)
            fingers_frame.addWidget(self.finger_widget[finger])    
        self.main_tab_layout.addLayout(fingers_frame)
        rospy.logwarn("inited tactile layout")

    def initialize_subscriber(self, side):
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
                self.finger_widget[finger].update_data(self._data[finger])

    def timerEvent(self):
        for finger in self._fingers:        
            self.finger_widget[finger].update_widget()


class BiotacVisualizationTab(GenericTabLayout):
    def __init__(self, tab_name, painter):
        super().__init__(tab_name, painter)

    def initialize_data_structure(self):
        self._data_fields = ['pac0', 'pac1', 'pac', 'pdc', 'tac', 'tdc', 'electrodes']
        self._data_representation_types = ["_visual", "_text"]
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = 0
       
    def init_tactile_layout(self):
        fingers_frame = QHBoxLayout()
        self.dot_widget = dict()
        for finger in self._fingers:
            fingers_frame.addWidget(self.create_finger_widget(finger))    
        self.main_tab_layout.addLayout(fingers_frame)

    def initialize_subscriber(self, side):
        self.sub = rospy.Subscriber('/{}/tactile'.format(side), BiotacAll, self._tactile_data_callback)

    def stop_subscribers(self, side):
        self.sub.unregister()

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
        #self.update()
        #self.dot_widget['ff'].set_new_values()

    def paintEvent(self, e):
        self.dot_widget['ff'].paintEvent(e)
        
    def create_finger_widget(self, finger):                              
        finger_frame = QGroupBox(finger)
        finger_frame.setAlignment(Qt.AlignHCenter)
        finger_frame_layout = QHBoxLayout()
        
        self.dot_widget[finger] = DotUnit(self.painter, "test{}".format(finger),0,0)

        finger_frame_layout.addWidget(self.dot_widget[finger].get_unit())
        finger_frame.setLayout(finger_frame_layout)
        return finger_frame




class VisualBiotac():
    def __init__(self, index, color, r = 0.1, version="v1"):
        self.index = index
        self.color = color
        self.x = position_x
        self.y = position_y
        self.r = r
        self.version = version

        self.coordinates = dict()
        self.coordinates['v1'] = dict()
        self.coordinates['v1']['factor'] = 17.5
        self.coordinates['v1']['sensing'] = dict()        
        self.coordinates['v1']['sensing']['x'] = [6.45, 3.65, 3.65, 6.45, 3.65, 6.45, 0.00, 1.95, -1.95, 0.00, 
                                                  -6.45, - 3.65, -3.65, -6.45, -3.65, -6.45, 0.00, 0.00, 0.00]
        self.coordinates['v1']['sensing']['y'] = [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38, 6.38, 8.38, 
                                                  7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38, 18.38, 22.18]
        self.coordinates['v1']['excitation'] = dict()
        self.coordinates['v1']['excitation']['x'] = [6.45, 3.75, -3.75, -6.45]
        self.coordinates['v1']['excitation']['y'] = [12.48, 24.48, 24.48, 12.48]


        self.coordinates['v2'] = dict()
        self.coordinates['v2']['factor'] = 25
        self.coordinates['v2']['sensing'] = dict()
        self.coordinates['v2']['sensing']['x'] = [5.00, 3.65, 6.45, 4.40, 2.70, 6.45, 4.40, 1.50, 4.00, 4.50, -5.00,
                                                 -3.65, -6.45, -4.40, -2.70, -6.45, -4.40, -1.50, -4.00, -4.50, 0.00,
                                                  1.95, -1.95, 0.00]
        self.coordinates['v2']['sensing']['y'] = [4.38, 6.38, 14.78, 15.50, 18.50, 19.08, 20.00, 21.00, 23.00, 25.00,
                                                  4.38, 6.38, 14.78, 15.50, 18.50, 19.08, 20.00, 21.00, 23.00, 25.00,
                                                  7.38, 11.50, 11.50, 15.20]
        self.coordinates['v2']['excitation']['x'] = [5.30, 6.00, -5.30, -6.00]
        self.coordinates['v2']['excitation']['y'] = [9.00, 22.00, 9.00, 22.00]


        self.sensing_electrodes_v2_x = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_x_locations",
                [5.00, 3.65, 6.45, 4.40, 2.70, 6.45, 4.40, 1.50, 4.00, 4.50,
                 -5.00, - 3.65, -6.45, -4.40, -2.70, -6.45, -4.40, -1.50, -4.00, -4.50,
                 0.00, 1.95, -1.95, 0.00])  # Physical electrode locations on the sensor
        self.sensing_electrodes_v2_y = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_y_locations",
                [4.38, 6.38, 14.78, 15.50, 18.50, 19.08, 20.00, 21.00, 23.00, 25.00,
                 4.38, 6.38, 14.78, 15.50, 18.50, 19.08, 20.00, 21.00, 23.00, 25.00,
                 7.38, 11.50, 11.50, 15.20])

        self.excitation_electrodes_v2_x = \
            rospy.get_param(
                "sr_gui_biotac/excitation_electrodes_x_locations",
                [5.30, 6.00, -5.30, -6.00])
        self.excitation_electrodes_v2_y = \
            rospy.get_param(
                "sr_gui_biotac/excitation_electrodes_y_locations",
                [9.00, 22.00, 9.00, 22.00])
    
    def draw_electrodes(self, painter):
        pass