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

from python_qt_binding.QtGui import  QColor, QPainter, QPaintDevice
from python_qt_binding.QtCore import Qt, QTimer, QRectF, QPoint, QSize
import numpy as np

from python_qt_binding.QtWidgets import (
    QFormLayout,
    QPushButton,
    QWidget,
    QGroupBox,
    QVBoxLayout,
    QLabel,
    QTextEdit,
    QPushButton
)

class CircleDot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.data = 0
        self.color = QColor(0,0,0)
        self.setMinimumSize(50,50)

    def _value_to_color(self, value):
        raise NotImplementedError("The function get_data must be implemented, with a dictionary in return")

    def set_thresholds(self, thresholds):
        self.thresholds = thresholds

    def update_color_dot(self, value):
        self.color = self._value_to_color(value)

    def paintEvent(self,event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(self.color)
        painter.setBrush(self.color)
        h = self.frameSize().height()
        w = self.frameSize().width()
        center = QPoint(w/2, h/2)        
        r = min(np.sqrt(h*h+w*w)/8,min(h,min(w,np.sqrt(h*h+w*w)/4)))
        painter.drawEllipse(center, r, r)


class DotUnitPST(QWidget):
    def __init__(self, finger, parent=None):
        super().__init__(parent=parent)  

        self.finger = finger
        self.data_fields = ['pressure', 'temperature']   
        self.data = dict()  
        self.dot = CircleDot(self)
        self.dot._value_to_color = self.value_to_color
        self.pst_range = [250, 850]
        self.initialize_data_structure()
        self.init_ui()        
    
    def value_to_color(self, value):
        pst_min = self.pst_range[0]
        pst_max = self.pst_range[1]

        threshold = list(range(pst_min, pst_max+1, int((pst_max-pst_min)/4)))

        r,g,b = 255,0,0

        if value < 100:  #indicate its broken
            r,g,b = 128,128,128

        if value <= threshold[0]:
            r = 0
            g = 0
            b = 255
        elif value < threshold[1]:
            r = 0
            g = 255 * ((value - threshold[0]) / (threshold[1] - threshold[0]))
            b = 255
        elif value < threshold[2]:
            r = 0
            g = 255
            b = 255 * ((threshold[2] - value) / (threshold[2] - threshold[1]))
        elif value < threshold[3]:
            r = 255 * ((value - threshold[2]) / (threshold[3] - threshold[2]))
            g = 255
            b = 0
        elif value < threshold[4]:
            r = 255
            g = 255 * ((threshold[4] - value) / (threshold[4] - threshold[3]))
            b = 255

        return QColor(r,g,b)

    def initialize_data_structure(self):          
        for data_field in self.data_fields:
            self.data[data_field] = 0

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignCenter)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        data_layout = QFormLayout()        
        data_layout.addRow(self.dot)
        self.label = dict()
        
        for data_field in self.data_fields:
            self.label[data_field] = QLabel("-")
            data_layout.addRow(QLabel(str(data_field[0])+":"), self.label[data_field])    
            
        dot_frame = QGroupBox(self.finger) 
        dot_frame.setAlignment(Qt.AlignCenter)
        dot_frame.setLayout(data_layout)

        main_layout.addWidget(dot_frame)
        self.setLayout(main_layout)

    def update_data(self, data):
        self.data = data

    def update_widget(self):
        for data_field in self.data_fields:
            self.label[data_field].setText(str(self.data[data_field]))
        self.dot.update_color_dot(self.data['pressure'])


