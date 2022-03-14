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


from python_qt_binding.QtGui import QColor, QFont
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QFormLayout,
    QLabel
)

from sr_gui_fingertip_visualization.generic_tactile_point import TactilePointGeneric


class TactilePointPST(TactilePointGeneric):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self._data_fields = ['pressure', 'temperature']
        self.get_dot().resize_dot(25)    
        self._init_widget()

    def _value_to_color(self, value):
        max_color, min_color = 255, 0
        pst_max, pst_min = 300, 800
        value = max(min_color, min(max_color, (value - pst_min)*(max_color-min_color) /
                                   (pst_max - pst_min) + min_color))
        return QColor(255-value, 0, value)

    def _init_widget(self):
        data_layout = QFormLayout()
        data_layout.addRow(self.get_dot())
        self._label = dict()

        for data_field in self._data_fields:
            self._label[data_field] = QLabel("-")
            data_layout.addRow(QLabel(f"{data_field[0]}: "), self._label[data_field])
        self.setLayout(data_layout)

    def update_data(self, data):
        for data_field in self._data_fields:
            self._label[data_field].setText(str(data[data_field]))
        self.get_dot().set_color(self._value_to_color(data['pressure']))
        self.get_dot().update()


class TactilePointBiotacSPMinus(TactilePointGeneric):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self._data_fields = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']
        self.get_dot().resize_dot(20)
        self._init_widget()

    def _value_to_color(self, value):
        # to change value (1000 and 200) as its taken from the topic
        r = min(255, max(0, 255 * (value - 1000) / 200))
        g = 0
        b = 255 - r
        return QColor(r, g, b)

    def _init_widget(self):
        widget_layout = QFormLayout()
        widget_layout.setFormAlignment(Qt.AlignRight)
        widget_layout.setLabelAlignment(Qt.AlignRight)

        widget_layout.addRow(self.get_dot())

        self._data_labels = dict()
        for data_field in self._data_fields:
            self._data_labels[data_field] = QLabel("-")
            self._data_labels[data_field].setMinimumSize(40, 10)
            self._data_labels[data_field].setSizePolicy(2, 2)
            widget_layout.addRow(QLabel(data_field+":"), self._data_labels[data_field])
        self.setLayout(widget_layout)

    def update_data(self, data):
        for data_field in self._data_fields:
            self._data_labels[data_field].setText(str(data[data_field]))
        self.get_dot().set_color(self._value_to_color(data['pdc']))
        self.get_dot().update()


class TactilePointBiotacSPPlus(TactilePointGeneric):
    def __init__(self, electrode_index, parent=None):
        super().__init__(index=electrode_index, parent=parent)
        self.electrode_index = electrode_index
        self._init_widget()

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
        widget_layout = QFormLayout()
        widget_layout.setLabelAlignment(Qt.AlignCenter)
        widget_layout.setFormAlignment(Qt.AlignCenter)

        self._electrode_label = QLabel("-", alignment=Qt.AlignCenter)
        self._electrode_label.setFont(QFont('Arial', 8))

        widget_layout.addRow(self.get_dot())
        widget_layout.addRow(self._electrode_label)

        self.setLayout(widget_layout)

    def update_data(self, data):
        self._electrode_label.setText(str(data['electrode']))
        #check change data['electrode'] <- data
        self.get_dot().set_color(self._value_to_color(data['electrode']))
        self.get_dot().update()
