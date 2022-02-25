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

from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtCore import Qt

from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QGridLayout,
    QRadioButton,
    QHBoxLayout,
    QVBoxLayout,
    QLineEdit,
    QGroupBox,
    QProgressBar
)


class GenericTabOptions(QWidget):
    def __init__(self, tab_name, parent=None):
        super().__init__(parent=parent)
        
        # autodetection
        self._fingers = ["th", "ff","mf","rf","lf"]

        self.finger_widgets = dict()
        widget_content = ["vis_pressure","vis_temp","text_pressue","text_temp",]
        for finger in self._fingers:
            self.finger_widgets[finger] = dict()
            for c in widget_content:
                self.finger_widgets[finger][c] = QWidget()
    
        self.init_generic_layout()
        self.init_tactile_layout()   
        self.setLayout(self.main_tab_layout)

    def init_generic_layout(self):
        self.main_tab_layout = QHBoxLayout()
        self.main_tab_layout.setContentsMargins(0, 0, 0, 0)
    
    def init_tactile_layout(self):
        raise NotImplementedError("The function create_tab_options must be implemented")

class PSTVisualizationTabOptions(GenericTabOptions):
    def __init__(self, tab_name, parent=None):
        super().__init__(tab_name, parent)
        self.pst_data = dict()

    def init_tactile_layout(self):
        fingers_frame = QHBoxLayout()
        for finger in self._fingers:
            fingers_frame.addWidget(self.create_finger_widget(finger))    
        self.main_tab_layout.addLayout(fingers_frame)


    def create_finger_widget(self, finger):        
       
        finger_layout = QVBoxLayout()

        self.finger_widgets[finger]['vis_pressure'] = QLineEdit("visual_pressure-{}".format(finger))
        self.finger_widgets[finger]['vis_temp'] = QLineEdit("visual_temp-{}".format(finger))
        self.finger_widgets[finger]['text_pressue'] = QLineEdit("text_pressure-{}".format(finger))
        self.finger_widgets[finger]['text_temp'] = QLineEdit("text_temp-{}".format(finger))

        finger_layout.addWidget(self.finger_widgets[finger].get('vis_pressure'))
        finger_layout.addWidget(self.finger_widgets[finger].get('vis_temp'))
        finger_layout.addWidget(self.finger_widgets[finger].get('text_pressue'))
        finger_layout.addWidget(self.finger_widgets[finger].get('text_temp'))       

        finger_frame = QGroupBox(finger)
        finger_frame.setAlignment(Qt.AlignHCenter)
        finger_frame.setLayout(finger_layout)

        return finger_frame
    
