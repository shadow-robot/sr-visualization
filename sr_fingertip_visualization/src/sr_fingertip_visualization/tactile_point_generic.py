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

from python_qt_binding.QtGui import QColor, QPainter
from python_qt_binding.QtCore import QPoint
from python_qt_binding.QtWidgets import QWidget
import rospy


class TactilePoint(QWidget):

    _MIN_SIZE_X = 50
    _INIT_RADIUS = 11

    def __init__(self, index=None, parent=None):
        super().__init__(parent=parent)
        self._color = QColor(0, 0, 0)
        self._text_color = QColor(0, 0, 0)
        self._painter = QPainter()
        self._index = str(index)

        self._radius = self._INIT_RADIUS
        self._title_height = 0
        self._center = QPoint(self.frameSize().width()/2, self._radius + self._title_height)
        self.setMinimumSize(self._MIN_SIZE_X, 2*self._radius)

    def paintEvent(self, event):
        self._painter.begin(self)
        self._painter.setRenderHint(QPainter.Antialiasing)
        self._painter.setPen(self._color)
        self._painter.setBrush(self._color)

        self.update_center(QPoint(self.frameSize().width()/2, self._radius + self._title_height))

        # The substraction of -1 prevents from coverting the dot with the frame
        self._painter.drawEllipse(self._center, self._radius - 1, self._radius - 1)
        if self._index.isnumeric():
            self._painter.setPen(QColor(0, 0, 0))
            # The +/-5 is to move the number closer to the center of the dot
            self._painter.drawText(self._center.x() - 5, self._center.y() + 5, self._index)
        self._painter.end()

    def update_center(self, qpoint):
        self._center = QPoint(qpoint)

    def set_color(self, qcolor):
        self._color = qcolor

    def set_text_color(self, qcolor):
        self._text_color = qcolor

    def resize_dot(self, radius):
        self._radius = radius
        self.update_center(QPoint(self.frameSize().width()/2, self._radius + self._title_height))
        self.setMinimumSize(self._MIN_SIZE_X, 2*self._radius)


class TactilePointGeneric(QWidget):
    def __init__(self, parent, index=None):
        super().__init__(parent=parent)
        self._dot = TactilePoint(index=index, parent=self)

    def get_data_fields(self):
        return self._CONST_DATA_FIELDS

    def _init_widget(self):
        raise NotImplementedError

    def _value_to_color(self):
        raise NotImplementedError

    def get_dot(self):
        return self._dot

    def update_data(self):
        raise NotImplementedError
