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

from python_qt_binding.QtGui import QColor, QFont
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QFormLayout,
    QLabel
)

from sr_fingertip_visualization.tactile_point_generic import TactilePointGeneric


class TactilePointPST(TactilePointGeneric):

    CONST_DATA_FIELDS = ['pressure', 'temperature']

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.get_dot().resize_dot(25)
        self._init_widget()

    def _value_to_color(self, value):
        # pylint: disable=W0221
        max_color, min_color = 255, 0
        pst_max, pst_min = 300, 800
        value = max(min_color, min(max_color, (value - pst_min)*(max_color-min_color) /
                                   (pst_max - pst_min) + min_color))
        return QColor(255-value, 0, value)

    def _init_widget(self):
        data_layout = QFormLayout()
        data_layout.addRow(self.get_dot())
        self._label = {}

        for data_field in self.CONST_DATA_FIELDS:
            self._label[data_field] = QLabel("-")
            data_layout.addRow(QLabel(f"{data_field[0]}: "), self._label[data_field])
        self.setLayout(data_layout)

    def update_data(self, data):
        # pylint: disable=W0221
        for data_field in self.CONST_DATA_FIELDS:
            self._label[data_field].setText(str(data[data_field]))
        self.get_dot().set_color(self._value_to_color(data['pressure']))
        self.get_dot().update()


class TactilePointBiotacSPMinus(TactilePointGeneric):

    CONST_DATA_FIELDS = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.get_dot().resize_dot(20)
        self._init_widget()

    def _value_to_color(self, value):
        # pylint: disable=W0221
        # to change values (1000 and 200), experimentally assigned for now but
        # need to be verified and changed after agreeing with production
        red = min(255, max(0, 255 * (value - 1000) / 200))
        green = 0
        blue = 255 - red
        return QColor(red, green, blue)

    def _init_widget(self):
        widget_layout = QFormLayout()
        widget_layout.setFormAlignment(Qt.AlignRight)
        widget_layout.setLabelAlignment(Qt.AlignRight)

        widget_layout.addRow(self.get_dot())

        self._data_labels = {}
        for data_field in self.CONST_DATA_FIELDS:
            self._data_labels[data_field] = QLabel("-")
            self._data_labels[data_field].setMinimumSize(40, 10)
            self._data_labels[data_field].setSizePolicy(2, 2)
            widget_layout.addRow(QLabel(data_field+":"), self._data_labels[data_field])
        self.setLayout(widget_layout)

    def update_data(self, data):
        # pylint: disable=W0221
        for data_field in self.CONST_DATA_FIELDS:
            self._data_labels[data_field].setText(str(data[data_field]))
        self.get_dot().set_color(self._value_to_color(data['pdc']))
        self.get_dot().update()


class TactilePointBiotacSPPlus(TactilePointGeneric):

    CONST_DATA_FIELDS = ['pac0', 'pac1', 'pdc', 'tac', 'tdc', 'electrodes', 'pac']

    def __init__(self, electrode_index, parent=None):
        super().__init__(index=electrode_index, parent=parent)
        self._init_widget()

    def _value_to_color(self, value):
        # pylint: disable=W0221
        red = 0.0
        green = 0.0
        blue = 255.0
        value = float(value)
        # values taken from previous version of the plugin
        # this is to have a specific color indication
        threshold = (0.0, 1000.0, 2000.0, 3000.0, 4095.0)
        if value <= threshold[0]:
            pass
        elif value < threshold[1]:
            red = 255
            green = 255 * (value - threshold[0]) / (threshold[1] - threshold[0])
            blue = 0
        elif value < threshold[2]:
            red = 255 * ((threshold[2] - value) / (threshold[2] - threshold[1]))
            green = 255
            blue = 0
        elif value < threshold[3]:
            red = 0
            green = 255
            blue = 255 * ((value - threshold[2]) / (threshold[3] - threshold[2]))
        elif value < threshold[4]:
            red = 0
            green = 255 * ((threshold[4] - value) / (threshold[4] - threshold[3]))
            blue = 255
        return QColor(red, green, blue)

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
        # pylint: disable=W0221
        self._electrode_label.setText(str(data))
        self.get_dot().set_color(self._value_to_color(data))
        self.get_dot().update()
