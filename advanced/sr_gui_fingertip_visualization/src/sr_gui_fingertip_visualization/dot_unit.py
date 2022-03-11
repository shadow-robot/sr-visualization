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

from python_qt_binding.QtGui import QColor, QPainter, QFont
from python_qt_binding.QtCore import Qt,QPoint
import numpy as np

from python_qt_binding.QtWidgets import (
    QFormLayout,
    QWidget,
    QVBoxLayout,
    QLabel
)


class CircleDot(QWidget):

    MIN_SIZE_X = 50

    def __init__(self, index=None, parent=None):
        super().__init__(parent=parent)
        self._color = QColor(0, 0, 0)
        self._painter = QPainter(self)
        self._index = str(index)

        self._radius = 11
        self._title_height = 0
        self._center = QPoint(self.frameSize().width()/2, self._radius + self._title_height)
        self.setMinimumSize(self.MIN_SIZE_X, 2*self._radius)

    def paintEvent(self, event):
        self._painter.begin(self)
        self._painter.setRenderHint(QPainter.Antialiasing)
        self._painter.setPen(self._color)
        self._painter.setBrush(self._color)

        self._center.setX(self.frameSize().width()/2)
        self._center.setY(self._radius + self._title_height)

        self._painter.drawEllipse(self._center, self._radius - 1, self._radius - 1)
        if self._index.isnumeric():
            self._painter.setPen(QColor(0, 0, 0))
            self._painter.drawText(self._center.x() - 5, self._center.y() + 5, self._index)
        self._painter.end()

    def setColor(self, color):
        self._color = color


class DotUnitGeneric(QWidget):
    def __init__(self, parent, index=None):
        super().__init__(parent=parent)
        self._dot = CircleDot(index=index, parent=self)

    def resize_dot(self, radius):
        self._dot._radius = radius
        self._dot._center = QPoint(self._dot.frameSize().width()/2, self._dot._radius + self._dot._title_height)
        self._dot.setMinimumSize(self._dot.MIN_SIZE_X, 2*radius)

    def _initialize_data_structure(self, **optional_parameters):
        raise NotImplementedError

    def _init_widget(self):
        raise NotImplementedError

    def _value_to_color(self, value):
        raise NotImplementedError

    def get_dot(self):
        return self._dot

    def update_data(self, value):
        raise NotImplementedError


class DotUnitPST(DotUnitGeneric):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self._initialize_data_structure()
        self.resize_dot(25)
        self._init_widget()

    def _value_to_color(self, value):
        max_color, min_color = 255, 0
        value = max(min_color, min(max_color, (value - self._pst_min)*(max_color-min_color) /
                                   (self._pst_max - self._pst_min) + min_color))
        return QColor(255-value, 0, value)

    def _initialize_data_structure(self):
        self._data = dict()
        self._data_fields = ['pressure', 'temperature']
        self._pst_max, self._pst_min = 300, 800
        for data_field in self._data_fields:
            self._data[data_field] = 0

    def _init_widget(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        data_layout = QFormLayout()
        data_layout.addRow(self.get_dot())
        self.label = dict()

        for data_field in self._data_fields:
            self.label[data_field] = QLabel("-")
            data_layout.addRow(QLabel(str(data_field[0])+":"), self.label[data_field])
        self.setLayout(data_layout)

    def update_data(self, data):
        for data_field in self._data_fields:
            self.label[data_field].setText(str(data[data_field]))
        self._dot.setColor(self._value_to_color(data['pressure']))
        self.get_dot().update()


class DotUnitBiotacSPMinus(DotUnitGeneric):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self._initialize_data_structure()
        self.resize_dot(20)
        self._init_widget()

    def _initialize_data_structure(self):
        self._data_fields = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']

    def _value_to_color(self, value):
        # to change value (1000 and 200) as its taken from the topic
        r = min(255, max(0, 255 * (value - 1000) / 200))
        g = 0
        b = 255 - r
        self._dot.setColor(QColor(r, g, b))

    def _init_widget(self):
        widget_layout = QFormLayout()
        widget_layout.setFormAlignment(Qt.AlignRight)
        widget_layout.setLabelAlignment(Qt.AlignRight)

        widget_layout.addRow(self.get_dot())

        self.data_labels = dict()
        for data_field in self._data_fields:
            self.data_labels[data_field] = QLabel("-")
            self.data_labels[data_field].setMinimumSize(40, 10)
            self.data_labels[data_field].setSizePolicy(2, 2)
            widget_layout.addRow(QLabel(data_field+":"), self.data_labels[data_field])

        #self.setMinimumSize(100, 200)
        self.setLayout(widget_layout)

    def update_data(self, data):
        for data_field in self._data_fields:
            self.data_labels[data_field].setText(str(data[data_field]))
        self.get_dot().update_color_dot(data['pdc'])  # to be changed from 'pdc' to the correct value
        self.get_dot().update()


class DotUnitBiotacSPPlus(DotUnitGeneric):
    def __init__(self, electrode_index, parent=None):
        super().__init__(index=electrode_index, parent=parent)
        self._initialize_data_structure(electrode_index)
        self._init_widget()

    def _initialize_data_structure(self, electrode_index):
        self.electrode_index = electrode_index

    def _value_to_color(self, value):
        r = 0.0
        g = 0.0
        b = 255.0
        value = float(value)
        threshold = (0.0, 1000.0, 2000.0, 3000.0, 4095.0)
        if value <= threshold[0]:
            pass
        elif value < threshold[1]:
            r = 255
            g = 255 * (value - threshold[0]) / (threshold[1] - threshold[0])
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
        return QColor(r, g, b)

    def _init_widget(self):
        main_layout = QVBoxLayout()
        widget_layout = QFormLayout()
        widget_layout.setLabelAlignment(Qt.AlignCenter)
        widget_layout.setFormAlignment(Qt.AlignCenter)

        self.electrode_label = QLabel("-", alignment=Qt.AlignCenter)
        self.electrode_label.setFont(QFont('Arial', 8))

        widget_layout.addRow(self.get_dot())
        widget_layout.addRow(self.electrode_label)

        self.setLayout(widget_layout)

    def update_data(self, data):
        self.electrode_label.setText(str(data))
        self.get_dot().update_color_dot(data)
        self.get_dot().update()
