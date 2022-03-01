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

from python_qt_binding.QtGui import  QColor, QPainter
from python_qt_binding.QtCore import Qt, QTimer, QRectF
from python_qt_binding import QtCore

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

class DotUnit(QWidget):
    def __init__(self, name, x, y, r = 100):
        super().__init__()
        self.name = name
        self.dot_x = int(x)
        self.dot_y = int(x) 
        self.r = r 
        self.colour = None
        self.dot_area = QRectF(self.dot_x, self.dot_y, self.dot_x, self.dot_x)      
        self.threshold = [0.0, 1000.0, 2000.0, 3000.0, 4095.0]

        self.finger_box = QHBoxLayout()
        self.finger_box.addWidget(QPushButton(name))

        self.timer = QTimer()
        self.timer.timeout.connect(self.paintEvent)

    def paintEvent(self):        
        self.painter = QPainter(self)
        self.painter.setBrush(self._value_to_colour(100))
        self.painter.drawRect(self.rect())
        rospy.logwarn("repainted")
    
    def get_layout(self):
        return self.finger_box
