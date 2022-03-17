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
import rospy
from enum import Enum

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QPushButton,
    QWidget,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QFormLayout,
    QLabel,
    QComboBox,
    QStackedLayout
)

from sr_gui_fingertip_visualization.tactile_points import (
    TactilePointPST,
    TactilePointBiotacSPPlus,
    TactilePointBiotacSPMinus
)
from sr_gui_fingertip_visualization.tab_layouts_generic import GenericTabLayout
from sr_robot_msgs.msg import ShadowPST, BiotacAll


class FingerWidgetVisualPST(QGroupBox):
    def __init__(self, side, finger, parent):
        super().__init__(parent=parent)
        self._tactile_point_widget = TactilePointPST(self)
        self._fingers = ['ff', 'mf', 'rf', 'lf', 'th']
        self._data = dict()
        self._finger = finger
        self._side = side
        self._timer = QTimer()
        self._subscriber = None

        self.setTitle(finger)
        self.setCheckable(True)
        self.setChecked(False)
        self.setSizePolicy(1, 1)
        self.clicked.connect(self.refresh)

        layout = QVBoxLayout()
        layout.addWidget(self._tactile_point_widget, alignment=Qt.AlignCenter)
        self._tactile_data_callback(rospy.wait_for_message('/{}/tactile'.format(self._side), ShadowPST))
        self.setLayout(layout)

    def refresh(self, state):
        if state:
            self.start_timer_and_subscriber()
        elif not state:
            self.stop_timer_and_subscriber()

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
            if finger == self._finger:
                for data_field in self._tactile_point_widget.get_data_fields():
                    if data_field == "pressure":
                        self._data[data_field] = data.pressure[i]
                    elif data_field == "temperature":
                        self._data[data_field] = data.temperature[i]

    def timerEvent(self):
        self._tactile_point_widget.update_data(self._data)


class BiotacSPPlusInfo(QGroupBox):
    def __init__(self):
        super().__init__()
        self._text_fields = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']
        self._data = dict.fromkeys(self._text_fields, 0)
        self._labels = dict()

        self.setSizePolicy(1, 2)
        self.setTitle("Data")

        layout_pressure = QHBoxLayout()
        layout_temperature = QHBoxLayout()

        for key in self._text_fields:
            self._labels[key] = QLabel(self)
            self._labels[key].setText(f"{key}:0")
            if key[0] == 'p':
                layout_pressure.addWidget(self._labels[key])
            elif key[0] == 't':
                layout_temperature.addWidget(self._labels[key])

        layout = QVBoxLayout()
        layout.addLayout(layout_pressure)
        layout.addLayout(layout_temperature)
        self.setLayout(layout)

    def update_values(self, data):
        common_keys = list(set(data.keys()) & set(self._text_fields))
        for key in common_keys:
            self._data[key] = data[key]

    def get_widget(self):
        return widget

    def refresh(self):
        for key in self._text_fields:
            self._labels[key].setText(f"{key}:{self._data[key]}")


class FingerWidgetVisualBiotacSPMinus(QGroupBox):
    def __init__(self, side, finger, parent):
        super().__init__(parent=parent)
        self._tactile_point_widget = TactilePointBiotacSPMinus(self)
        self._fingers = ['ff', 'mf', 'rf', 'lf', 'th']
        self._data = dict()
        self._finger = finger
        self._side = side
        self._timer = QTimer()
        self._subscriber = None

        self.setTitle(finger)
        self.setCheckable(True)
        self.setChecked(False)
        self.setSizePolicy(1, 1)
        self.clicked.connect(self.refresh)

        layout = QVBoxLayout()
        layout.addWidget(self._tactile_point_widget, alignment=Qt.AlignCenter)
        self._tactile_data_callback(rospy.wait_for_message('/{}/tactile'.format(self._side), BiotacAll))
        self.setLayout(layout)

    def refresh(self, state):
        if state:
            self.start_timer_and_subscriber()
        elif not state:
            self.stop_timer_and_subscriber()

    def start_timer_and_subscriber(self):
        self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), BiotacAll, self._tactile_data_callback)
        self._timer.timeout.connect(self.timerEvent)
        self._timer.start(10)

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            if finger == self._finger:
                for data_field in self._tactile_point_widget.get_data_fields():
                    if data_field == "pac0":
                        self._data[data_field] = data.tactiles[i].pac0
                    elif data_field == "pac1":
                        self._data[data_field] = data.tactiles[i].pac1
                    elif data_field == "pdc":
                        self._data[data_field] = data.tactiles[i].pdc
                    elif data_field == "tac":
                        self._data[data_field] = data.tactiles[i].tac
                    elif data_field == "tdc":
                        self._data[data_field] = data.tactiles[i].tdc

    def timerEvent(self):
        self._tactile_point_widget.update_data(self._data)


class FingerWidgetVisualBiotacSPPlus(QGroupBox):
    def __init__(self, side, finger, parent):
        super().__init__(parent=parent)
        self._fingers = ['ff', 'mf', 'rf', 'lf', 'th']
        self._version = 'v2'
        self._data = dict()
        self._finger = finger
        self._side = side
        self._timer = QTimer()
        self._data_bar = BiotacSPPlusInfo()
        self._datatype_to_display = 'electrodes'
        self._subscriber = None

        self.setTitle(finger)
        self.setCheckable(True)
        self.setChecked(False)
        self.setSizePolicy(1, 1)
        self.clicked.connect(self.refresh)

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

        x_cords = self._coordinates[self._version]['sensing']['x']
        y_cords = self._coordinates[self._version]['sensing']['y']
        min_x = abs(min(self._coordinates[self._version]['sensing']['x']))
        min_y = abs(min(self._coordinates[self._version]['sensing']['y']))
        max_x = abs(max(self._coordinates[self._version]['sensing']['x']))
        max_y = abs(max(self._coordinates[self._version]['sensing']['y']))
        y_offset = 2

        self._tactile_point_widget = list()
        container_widget = QWidget()
        container_widget.setMinimumSize((max_x) * 50, ((max_y) * 19))
        for i, (x, y) in enumerate(zip(x_cords, y_cords)):
            self._tactile_point_widget.append(TactilePointBiotacSPPlus(i, container_widget))
            self._tactile_point_widget[i].move((x + min_x) * 20, ((y - y_offset - min_y / 2) * 20))

        layout = QVBoxLayout()
        layout.addWidget(container_widget, alignment=Qt.AlignCenter)
        layout.addWidget(self._data_bar)

        self._tactile_data_callback(rospy.wait_for_message('/{}/tactile'.format(self._side), BiotacAll))
        self._electrodes_to_display_count = len(self._data['electrodes'])
        self.setLayout(layout)

    def refresh(self, state):
        if state:
            self.start_timer_and_subscriber()
        elif not state:
            self.stop_timer_and_subscriber()

    def start_timer_and_subscriber(self):
        self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), BiotacAll, self._tactile_data_callback)
        self._timer.timeout.connect(self.timerEvent)
        self._timer.start(10)

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._fingers):
            if finger == self._finger:
                for data_field in self._tactile_point_widget[i].get_data_fields():
                    if data_field == "pac0":
                        self._data[data_field] = data.tactiles[i].pac0
                    elif data_field == "pac1":
                        self._data[data_field] = data.tactiles[i].pac1
                    elif data_field == "pac":
                        self._data[data_field] = list(data.tactiles[i].pac)
                    elif data_field == "pdc":
                        self._data[data_field] = data.tactiles[i].pdc
                    elif data_field == "tac":
                        self._data[data_field] = data.tactiles[i].tac
                    elif data_field == "tdc":
                        self._data[data_field] = data.tactiles[i].tdc
                    elif data_field == "electrodes":
                        self._data[data_field] = list(data.tactiles[i].electrodes)

    def timerEvent(self):
        for i in range(self._electrodes_to_display_count):
            self._tactile_point_widget[i].update_data(self._data[self._datatype_to_display][i])
        self._data_bar.update_values(self._data)
        self._data_bar.refresh()

    def get_datatype_to_display(self):
        return self._datatype_to_display

    def change_datatype_to_display(self, datatype):
        if datatype in ["pac", "electrodes"]:
            self._datatype_to_display = datatype
            self._electrodes_to_display_count = len(self._data[self._datatype_to_display])

        for i in range(len(self._tactile_point_widget)):
            if i < self._electrodes_to_display_count:
                self._tactile_point_widget[i].show()
            else:
                self._tactile_point_widget[i].hide()


class FingerWidgetVisualBiotacBlank(QGroupBox):
    def __init__(self, finger, parent):
        super().__init__(parent=parent)
        self.setTitle(finger)
        self.setCheckable(True)
        self.setChecked(False)
        layout = QHBoxLayout()
        no_tactile_label = QLabel()
        no_tactile_label.setText("No tactile sensor")
        layout.addWidget(no_tactile_label, alignment=Qt.AlignCenter)
        self.setLayout(layout)

    def start_timer_and_subscriber(self):
        pass

    def stop_timer_and_subscriber(self):
        pass
