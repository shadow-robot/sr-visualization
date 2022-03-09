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
    QStackedLayout,
    QLayout
)

from sr_gui_fingertip_visualization.dot_unit import DotUnitPST, DotUnitBiotacSPPlus, DotUnitBiotacSPMinus
from sr_robot_msgs.msg import ShadowPST, BiotacAll

from sr_utilities.hand_finder import HandFinder
from sr_hand.tactile_receiver import TactileReceiver


class GenericTabLayout(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        

        type_right = rostopic.get_topic_type("/rh/tactile")
        type_left = rostopic.get_topic_type("/lh/tactile")

        self._hand_ids = [i[1].split('/')[1] for i in [type_right, type_left] if i[1]]
        self._hand_ids.append('lh')

        self._subscriber = None 
        self._fingers = ["ff", 'mf', 'rf', 'lf', 'th']
        self._finger_frame = dict()
        self._finger_widgets = dict()
        self._timer = QTimer(self)
        self._init_generic_layout()

    def _init_generic_layout(self):
        self.main_tab_layout = QVBoxLayout()
        self.main_tab_layout.setAlignment(Qt.AlignTop)
        self.main_tab_layout.setContentsMargins(0, 0, 0, 0)
       
        rospy.logwarn("Inited generic layout elements")

    def _init_tactile_layout(self):
        raise NotImplementedError("The function _init_tactile_layout must be implemented")

    def _tactile_data_callback(self):
        raise NotImplementedError("The function _tactile_data_callback must be implemented")

    def _initialize_data_structure(self):
        raise NotImplementedError("The function _initialize_data_structure must be implemented")

    def get_finger_frames(self):
        return self._finger_frame


class PSTVisualizationTab(GenericTabLayout):
    def __init__(self, side, parent):
        super().__init__(parent)
        self._side = side
        self._initialize_data_structure()
        self._init_tactile_layout()

    def _initialize_data_structure(self):
        self._data_fields = ['pressure', 'temperature']
        self._data = dict()
        for finger in self._fingers:
            self._data[finger] = dict()
            for data_field in self._data_fields:
                self._data[finger][data_field] = 0

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
                if data_field == "pressure":
                    self._data[finger][data_field] = data.pressure[i]
                elif data_field == "temperature":
                    self._data[finger][data_field] = data.temperature[i]

    def _init_tactile_layout(self):
        fingers_frame = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            fingers_frame.addWidget(self._init_finger_widget(finger))
        self.main_tab_layout.addLayout(fingers_frame)
        rospy.logwarn("inited pst tactile layout")
        self.main_tab_layout.addLayout(fingers_frame)
        self.setLayout(self.main_tab_layout)

    def _init_finger_widget(self, finger):
        self._finger_frame[finger] = QGroupBox(finger)
        self._finger_frame[finger].setCheckable(True)
        self._finger_frame[finger].setSizePolicy(1, 1)

        self._finger_widgets[finger] = DotUnitPST(self._finger_frame[finger])

        layout = QHBoxLayout()
        layout.addWidget(self._finger_widgets[finger])
        self._finger_frame[finger].setLayout(layout)

        return self._finger_frame[finger]

    def timerEvent(self):
        for finger in self._fingers:
            self._finger_widgets[finger].update_data(self._data[finger])


class BiotacType(Enum):
    SP_PLUS = 0
    SP_MINUS = 1
    BLANK = 3


class BiotacVisualizationTab(GenericTabLayout):
    def __init__(self, side, parent):
        super().__init__(parent)
        self._side = side
        self._version = "v2"  # Autodetect
        self._electrode_count = 0
        self._selected_fingers = []
        self._initialize_data_structure()
        self._init_tactile_layout()
    
    def _initialize_data_structure(self):
        self._data_fields = ['pac0', 'pac1', 'pac', 'pdc', 'tac', 'tdc', 'electrodes']
        self._data = dict()
        for finger in self._fingers:
            self._data[finger] = dict()

        self._tactile_data_callback(rospy.wait_for_message('/{}/tactile'.format(self._side), BiotacAll))

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

    def _init_tactile_layout(self):
        self.finger_complete_layout = QHBoxLayout()
        for finger in ['th', 'ff', 'mf', 'rf', 'lf']:
            self.finger_complete_layout.addWidget(self._init_finger_widget(finger))
        self.main_tab_layout.addLayout(self.finger_complete_layout)
        self.setLayout(self.main_tab_layout)

    def _init_finger_widget(self, finger):
        self._finger_frame[finger] = QGroupBox(finger)
        self._finger_frame[finger].setCheckable(True)
        self._finger_frame[finger].setSizePolicy(1, 1)
        layout = QVBoxLayout()

        x_cords = self._coordinates[self._version]['sensing']['x']
        y_cords = self._coordinates[self._version]['sensing']['y']
        min_x = abs(min(self._coordinates[self._version]['sensing']['x']))
        min_y = abs(min(self._coordinates[self._version]['sensing']['y']))
        max_x = abs(max(self._coordinates[self._version]['sensing']['x']))
        max_y = abs(max(self._coordinates[self._version]['sensing']['y']))
        y_offset = 2

        detected_fingertip_type = self._detect_biotac_type(finger)

        if detected_fingertip_type == BiotacType.SP_PLUS:
            self._finger_widgets[finger] = [QWidget()] * len(self._coordinates[self._version]['sensing']['x'])
            container_widget = QWidget()
            container_widget.setMinimumSize((max_x) * 50, ((max_y) * 19))
            for i, (x, y) in enumerate(zip(x_cords, y_cords)):
                self._finger_widgets[finger][i] = DotUnitBiotacSPPlus(i, container_widget)
                self._finger_widgets[finger][i].move((x + min_x) * 20, ((y - y_offset - min_y / 2) * 20))
            layout.addWidget(container_widget, alignment=Qt.AlignCenter)

            rows = 4 
            columns = 5
            remaining_data = QGroupBox("Data")

            remaining_data_layout = QVBoxLayout()
            remaining_data_layout_pac = QGridLayout()

            self.pac_labels = list()
            #rospy.logwarn(len(self.pac_labels))
            for row in range(rows):
                for col in range(columns):                
                    self.pac_labels.append(QLabel("P:{}".format(len(self.pac_labels))))
                    remaining_data_layout_pac.addWidget(self.pac_labels[-1], row, col, alignment=Qt.AlignCenter)
            remaining_data_layout.addLayout(remaining_data_layout_pac)

            remaining_data_layout_pdc_tdc_tac = QGridLayout()
            remaining_data_layout_pdc_tdc_tac.addWidget(QLabel("tac"), 0, 0, alignment=Qt.AlignCenter)
            remaining_data_layout_pdc_tdc_tac.addWidget(QLabel("tdc"), 0, 1, alignment=Qt.AlignCenter)
            remaining_data_layout_pdc_tdc_tac.addWidget(QLabel("pac"), 0, 2, alignment=Qt.AlignCenter)

            remaining_data_layout.addLayout(remaining_data_layout_pdc_tdc_tac)
            remaining_data.setLayout(remaining_data_layout)
            layout.addWidget(remaining_data)

        elif detected_fingertip_type == BiotacType.SP_MINUS:
            self._finger_widgets[finger] = DotUnitBiotacSPMinus(self._finger_frame[finger])
            layout.addWidget(self._finger_widgets[finger], alignment=Qt.AlignCenter)

        elif detected_fingertip_type == BiotacType.BLANK:
            self._finger_widgets[finger] = QLabel("No tactile sensor")
            layout.addWidget(self._finger_widgets[finger], alignment=Qt.AlignCenter)

        rospy.logwarn("Adding widget type: {} for finger {}".format(self._detect_biotac_type(finger), finger))
        self._finger_frame[finger].setLayout(layout)
        return self._finger_frame[finger]

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

    def start_timer_and_subscriber(self):
        self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), BiotacAll, self._tactile_data_callback)
        self._timer.timeout.connect(self.timerEvent)
        self._timer.start(10)        

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()

    def timerEvent(self):
        for finger in self._fingers:
            if type(self._finger_widgets[finger]) == list:
                for electrode in range(self._electrode_count):
                    if type(self._finger_widgets[finger][electrode]) == DotUnitBiotacSPPlus:
                        value = self._data[finger]['electrodes'][electrode]
                        self._finger_widgets[finger][electrode].update_data(value)
                        for i in range(len(self._data[finger]['pac'])):            
                            self.pac_labels[i].setText("P:{}".format(self._data[finger]['pac'][i]))

            elif type(self._finger_widgets[finger]) == DotUnitBiotacSPMinus:
                self._finger_widgets[finger].update_data(self._data[finger])


class VisualizationTab(GenericTabLayout):
    def __init__(self, parent):
        super().__init__(parent=parent)
        self._detect_hand_and_tactile_type()
        self._init_generic_layout()
        self._init_layout()
        self._create_connections()
        
    def _detect_hand_and_tactile_type(self):
        type_right = rostopic.get_topic_type("/rh/tactile")
        type_left = rostopic.get_topic_type("/lh/tactile")

        self._hand_ids = [i[1].split('/')[1] for i in [type_right, type_left] if i[1]]        
        self._types = [i[0].split('/')[1] for i in [type_right, type_left] if i[1]]     
        self._tactile_topics = dict(zip(self._hand_ids, self._types))

    def _init_layout(self):        

        self.finger_layout = QVBoxLayout(self)

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
        self.stacked_layout = QStackedLayout(self)

        self.fingertip_widget = dict()        
        for side, tactile_topic in self._tactile_topics.items():            
            if tactile_topic == "ShadowPST":
                self.fingertip_widget[side] = PSTVisualizationTab(side, self)
            elif tactile_topic == "BiotacAll":
                self.fingertip_widget[side] = BiotacVisualizationTab(side, self)
            self.stacked_layout.addWidget(self.fingertip_widget[side])

        self.finger_layout.addWidget(self.options_layout_groupbbox)
        self.finger_layout.addLayout(self.stacked_layout)        
        self.setLayout(self.finger_layout)

        self._current_side = list(self._tactile_topics.keys())[0]
        self._start_selected_widget(self.fingertip_widget[self._current_side])

    def _create_connections(self):
        self.hand_id_selection.currentIndexChanged.connect(self._combobox_action_hand_id_selection)    
        self.finger_selection_show_selected_button.clicked.connect(self._button_action_show_selected_fingers)
        self.finger_selection_show_all_button.clicked.connect(self._button_action_show_all)

    def _combobox_action_hand_id_selection(self):        
        self._current_side = self.hand_id_selection.currentText()
        self.stacked_layout.setCurrentWidget(self.fingertip_widget[self._current_side])
        self._start_selected_widget(self.fingertip_widget[self._current_side])
        rospy.logwarn(self._current_side)

    def _button_action_show_selected_fingers(self):
        fingertip_widgets = self.fingertip_widget[self._current_side].get_finger_frames()
        self._selected_fingers = [finger for finger in self._fingers if fingertip_widgets[finger].isChecked()]
        rospy.logwarn(self._selected_fingers)
        for finger in self._fingers:
            if finger in self._selected_fingers:
                fingertip_widgets[finger].show()
            else:
                fingertip_widgets[finger].hide()  

    def _button_action_show_all(self):
        fingertip_widgets = self.fingertip_widget[self._current_side].get_finger_frames()
        self._selected_fingers = [finger for finger in self._fingers if fingertip_widgets[finger].isChecked()]
        rospy.logwarn(self._selected_fingers)
        for finger in self._fingers:
            fingertip_widgets[finger].setChecked(True)
            fingertip_widgets[finger].show()


    def _start_selected_widget(self, selected_widget):
        for widget in self.fingertip_widget.values():
            if widget == selected_widget:
                widget.start_timer_and_subscriber()
            else:
                widget.stop_timer_and_subscriber()
 

    def changestack(self):
        self.stacked_layout.setCurrentIndex(self.i%2)
        #rospy.logwarn(self.)
        self.i += 1

