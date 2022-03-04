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

from python_qt_binding.QtGui import  QColor, QPainter, QPaintDevice, QFont
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
        self.painter = QPainter(self)
        self.painter.setViewport(0,0,50,50)
        self.setMinimumSize(50,50)

    def update_color_dot(self, value):
        r = 0.0
        g = 0.0
        b = 255.0
        value = float(value)
        threshold = (0.0, 1000.0, 2000.0, 3000.0, 4095.0)
        if value <= threshold[0]:
            pass
        elif value < threshold[1]:
            r = 255
            g = 255 * ((value - threshold[0]) / (threshold[1] - threshold[0]))            
            b = 0
        elif value < threshold[2]:
            r = 255 * ((threshold[2] - value) / (threshold[2] - threshold[1]))
            g = 255
            b = 0
        elif value < threshold[3]:
            r = 0
            g = 255
            b = 255 * ((value - threshold[2]) / (threshold[3] - threshold[2]))
        elif value < threshold[4]:
            r = 0
            g = 255 * ((threshold[4] - value) / (threshold[4] - threshold[3]))
            b = 255
        self.color = QColor(r, g, b)

    def paintEvent(self,event):
        self.painter.begin(self)        
        self.painter.setRenderHint(QPainter.Antialiasing)
        self.painter.setPen(self.color)
        self.painter.setBrush(self.color)
        h = self.frameSize().height()
        w = self.frameSize().width()
        r = min(h,w)/2
        center = QPoint(w/2, h/2)   
        self.painter.drawEllipse(center, r, r)
        self.painter.end()


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
        self.dot.update_color_dot(data)

class DotUnitBiotacSPMinus(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)  

        self.data = dict()  
        self.dot = CircleDot(self)
        self.dot.update_color_dot = self.update_color_dot
        self.initialize_data_structure()
        self.init_ui()        

    def update_color_dot(self, value):
        # to change value as its taken from the topic
        r = min(255,max(0, 255*(value-1000)/200))
        g = 0
        b = 255 - r
        self.dot.color = QColor(r,g,b)

    def initialize_data_structure(self):  
        self._data_fields = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']    
        for data_field in self._data_fields:
            self.data[data_field] = 0

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignCenter)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        data_layout = QFormLayout()        
        data_layout.addRow(self.dot)
        self.data_labels = dict()
        for data_field in self._data_fields:
            self.data_labels[data_field] = QLabel("-")
            self.data_labels[data_field].setMinimumSize(40,10)
            data_layout.addRow(QLabel(data_field+":"), self.data_labels[data_field])
                    
        dot_frame = QGroupBox("") 
        dot_frame.setAlignment(Qt.AlignCenter)
        dot_frame.setLayout(data_layout)

        main_layout.addWidget(dot_frame)
        self.setLayout(main_layout)

    def update_data(self, data):
        
        for data_field in self._data_fields:
            self.data_labels[data_field].setText(str(data[data_field]))
        self.dot.update_color_dot(data['pdc'])
        self.dot.update()


class DotUnitBiotac(QWidget):
    def __init__(self, electrode_index, parent=None):
        super().__init__(parent=parent)  

        self.dot = CircleDot(self)    
        self.dot.setMinimumSize(20,20)
        self.electrode_index = electrode_index
        
        self.initialize_data_structure()
        self.init_ui()        

    def initialize_data_structure(self): 
        self.data = 0

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignCenter)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        self.electrode_label = QLabel("E{}:{}".format(self.electrode_index, self.data))
        self.electrode_label.setFont(QFont('Arial',6))
        self.electrode_label.setMinimumSize(40,10)

        data_layout = QFormLayout()        
        data_layout.addRow(self.dot)          
        data_layout.addRow(self.electrode_label)               

        self.setLayout(data_layout)

    def update_data(self, data):
        self.data = data        
        self.dot.update_color_dot(data)
        self.electrode_label.setText("E{}:{}".format(self.electrode_index, self.data))
        self.dot.update()



