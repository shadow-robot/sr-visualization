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
import rospy
import rospkg
from sr_robot_msgs.msg import ShadowPST, BiotacAll


from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QIcon, QColor
from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QFormLayout,
    QLabel,
    QComboBox,
    QCheckBox,
    QStackedLayout
)


class GenericGraphTab(QWidget):
    def __init__(self, side, parent):
        super().__init__(parent=parent)

        self._side = side
        self._fingers = ['ff', 'mf', 'rf', 'lf', 'th']        
        self._data = dict()
        self._timer = QTimer()
        self._finger_widgets = dict()
        self._data_selection = dict()
        self._data_selection_checkboxes = dict()
        self._plot = dict()

        self._initialize_data_structure()

        ICON_DIR = os.path.join(rospkg.RosPack().get_path('sr_visualization_icons'), 'icons')
        self.ICONS = {
            'blue': QIcon(os.path.join(ICON_DIR, 'blue.png')),
            'red': QIcon(os.path.join(ICON_DIR, 'red.png')),
            'green': QIcon(os.path.join(ICON_DIR, 'green.png')),
            'magenta': QIcon(os.path.join(ICON_DIR, 'magenta.png')),
            'gray': QIcon(os.path.join(ICON_DIR, 'gray.png')),
            'cyan': QIcon(os.path.join(ICON_DIR, 'cyan.png'))
        }

        available_colors = list(self.ICONS.keys())
        self._legend_colors = dict()
        for i, data_field in enumerate(self._data_fields):
            self._legend_colors[data_field] = dict()
            self._legend_colors[data_field]['icon'] = self.ICONS[available_colors[i]]
            self._legend_colors[data_field]['plot_color'] = QColor(available_colors[i])

    def _initialize_data_structure(self):
        raise NotImplementedError("The function _initialize_data_structure must be implemented")

    def start_timer_and_subscriber(self):
        raise NotImplementedError("The function start_timer_and_subscriber must be implemented")

    def get_finger_widgets(self):
        return self._finger_widgets


class GenericTabLayout(QWidget):
    def __init__(self, parent):
        super().__init__(parent=parent)

        self._subscriber = None
        self._fingers = ["ff", 'mf', 'rf', 'lf', 'th']
        self._finger_widgets = dict()
        self._timer = QTimer(self)
    
    def _init_tactile_layout(self):
        raise NotImplementedError("The function _init_tactile_layout must be implemented")

    def _tactile_data_callback(self):
        raise NotImplementedError("The function _tactile_data_callback must be implemented")

    def _initialize_data_structure(self):
        raise NotImplementedError("The function _initialize_data_structure must be implemented")

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()

    def get_finger_widgets(self):
        return self._finger_widgets
