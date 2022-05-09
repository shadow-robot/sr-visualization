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

import os
import rospy
import rospkg
import yaml

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QWidget,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QLabel,
)
from sr_robot_msgs.msg import ShadowPST, BiotacAll
from diagnostic_msgs.msg import DiagnosticArray
from sr_fingertip_visualization.tactile_points import (
    TactilePointPST,
    TactilePointBiotacSPPlus,
    TactilePointBiotacSPMinus
)


class FingerWidget(QGroupBox):

    _CONST_FINGERS = ['ff', 'mf', 'rf', 'lf', 'th']

    def __init__(self, side, finger, parent=None):
        super().__init__(parent=parent)
        self._side = side
        self._finger = finger

        self.setTitle(f"{self._finger} ({self._get_serial_number()})")
        self.setCheckable(True)
        self.setChecked(False)
        self.setSizePolicy(1, 1)

    def _get_serial_number(self):
        serial_number = '----'
        tactile_index = self._CONST_FINGERS.index(self._finger) + 1
        expected_diagnostic_name = f"{self._side} Tactile {tactile_index}"  # example name: "rh Tactile 5"

        now = rospy.get_time()
        while rospy.get_time() - now < 2:
            # 2s to give more time for messages to arrive due to low publishing rate of /diagnostics
            try:
                diagnostic_msg = rospy.wait_for_message('/diagnostics', DiagnosticArray, timeout=1)
                diagnostic_data = [x for x in diagnostic_msg.status if x.name == expected_diagnostic_name]
                if diagnostic_data:
                    details_dict = {}
                    for entry in diagnostic_data[0].values:
                        details_dict.update({entry.key: entry.value})
                    serial_number = details_dict['Serial Number'][-4:]
                    break
            except rospy.exceptions.ROSException:
                break
        return serial_number


class FingerWidgetVisualPST(FingerWidget):

    _CONST_FINGERS = ['ff', 'mf', 'rf', 'lf', 'th']

    def __init__(self, side, finger, parent):
        super().__init__(side, finger, parent)
        self._tactile_point_widget = TactilePointPST(self)
        self._data = {}
        self._finger = finger
        self._side = side
        self._timer = QTimer()
        self._subscriber = None

        self.toggled.connect(self.refresh)

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
        if not self._subscriber:
            self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), ShadowPST,
                                                self._tactile_data_callback)
            self._timer.timeout.connect(self.timerEvent)
            self._timer.start(10)

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()
            self._subscriber = None

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._CONST_FINGERS):
            if finger == self._finger:
                for data_field in self._tactile_point_widget.get_data_fields():
                    if data_field == "pressure":
                        self._data[data_field] = data.pressure[i]
                    elif data_field == "temperature":
                        self._data[data_field] = data.temperature[i]

    def timerEvent(self):  # pylint: disable=C0103
        self._tactile_point_widget.update_data(self._data)


class BiotacSPPlusInfo(QGroupBox):

    _CONST_TEXT_FIELDS = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']

    def __init__(self):
        super().__init__()
        self._data = dict.fromkeys(self._CONST_TEXT_FIELDS, 0)
        self._labels = {}

        self.setSizePolicy(1, 2)
        self.setTitle("Data")

        for key in self._CONST_TEXT_FIELDS:
            self._labels[key] = QLabel(self)
            self._labels[key].setText(f"{key}: -")

        layout = QGridLayout()
        layout.addWidget(self._labels['pac0'], 0, 0, alignment=Qt.AlignLeft)
        layout.addWidget(self._labels['pac1'], 0, 1, alignment=Qt.AlignLeft)
        layout.addWidget(self._labels['pdc'], 0, 2, alignment=Qt.AlignLeft)
        layout.addWidget(self._labels['tac'], 1, 0, alignment=Qt.AlignLeft)
        layout.addWidget(self._labels['tdc'], 1, 1, alignment=Qt.AlignLeft)
        layout.addWidget(QLabel(), 1, 2, alignment=Qt.AlignLeft)

        self.setLayout(layout)

    def update_values(self, data):
        common_keys = [set(data.keys()) & set(self._CONST_TEXT_FIELDS)]
        for key in common_keys:
            self._data[key] = data[key]

    def refresh(self):
        for key in self._CONST_TEXT_FIELDS:
            self._labels[key].setText(f"{key}: {self._data[key]}")


class FingerWidgetVisualBiotacSPMinus(FingerWidget):

    _CONST_FINGERS = ['ff', 'mf', 'rf', 'lf', 'th']

    def __init__(self, side, finger, parent):
        super().__init__(side, finger, parent)
        self._tactile_point_widget = TactilePointBiotacSPMinus(self)
        self._data = {}
        self._finger = finger
        self._side = side
        self._timer = QTimer()
        self._subscriber = None

        self.toggled.connect(self.refresh)

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
        if not self._subscriber:
            self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), BiotacAll,
                                                self._tactile_data_callback)
            self._timer.timeout.connect(self.timerEvent)
            self._timer.start(10)

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()
            self._subscriber = None

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._CONST_FINGERS):
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

    def timerEvent(self):  # pylint: disable=C0103
        self._tactile_point_widget.update_data(self._data)


class FingerWidgetVisualBiotacSPPlus(FingerWidget):

    _CONST_FINGERS = ['ff', 'mf', 'rf', 'lf', 'th']

    def __init__(self, side, finger, parent):
        # pylint: disable=R0914
        super().__init__(side, finger, parent)
        self._version = 'v2'
        self._data = {}
        self._finger = finger
        self._side = side
        self._timer = QTimer()
        self._data_bar = BiotacSPPlusInfo()
        self._datatype_to_display = 'electrodes'
        self._subscriber = None

        self.toggled.connect(self.refresh)
        self._succeded_config_load = False

        try:
            config_dir = os.path.join(rospkg.RosPack().get_path('sr_fingertip_visualization'), 'config')
            with open(config_dir + "/tactile_point_cooridinates.yaml", 'r', encoding="ASCII") as stream:
                self._coordinates = yaml.safe_load(stream)
                self._succeded_config_load = True
        except FileNotFoundError:
            rospy.logerr("Config file not found!")
            layout = QVBoxLayout()
            layout.addWidget(QLabel("Error!"), alignment=Qt.AlignBottom | Qt.AlignHCenter)
            layout.addWidget(QLabel("Layout config file not found!"), alignment=Qt.AlignTop | Qt.AlignHCenter)
            self.setLayout(layout)
            return

        x_cords = self._coordinates[self._version]['sensing']['x']
        y_cords = self._coordinates[self._version]['sensing']['y']
        min_x = abs(min(self._coordinates[self._version]['sensing']['x']))
        min_y = abs(min(self._coordinates[self._version]['sensing']['y']))
        max_x = abs(max(self._coordinates[self._version]['sensing']['x']))
        max_y = abs(max(self._coordinates[self._version]['sensing']['y']))
        y_offset = 2

        self._tactile_point_widget = []
        container_widget = QWidget()
        container_widget.setMinimumSize((max_x) * 50, ((max_y) * 19))
        for i, (x_cord, y_cord) in enumerate(zip(x_cords, y_cords)):
            self._tactile_point_widget.append(TactilePointBiotacSPPlus(i, container_widget))
            self._tactile_point_widget[i].move((x_cord + min_x) * 20, ((y_cord - y_offset - min_y / 2) * 20))

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
        if self._succeded_config_load and not self._subscriber:
            self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), BiotacAll,
                                                self._tactile_data_callback)
            self._timer.timeout.connect(self.timerEvent)
            self._timer.start(10)

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()
            self._subscriber = None

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._CONST_FINGERS):
            if finger == self._finger:
                for data_field in self._tactile_point_widget[i].get_data_fields():
                    if data_field == "pac0":
                        self._data[data_field] = data.tactiles[i].pac0
                    elif data_field == "pac1":
                        self._data[data_field] = data.tactiles[i].pac1
                    elif data_field == "pac":
                        self._data[data_field] = [data.tactiles[i].pac]
                    elif data_field == "pdc":
                        self._data[data_field] = data.tactiles[i].pdc
                    elif data_field == "tac":
                        self._data[data_field] = data.tactiles[i].tac
                    elif data_field == "tdc":
                        self._data[data_field] = data.tactiles[i].tdc
                    elif data_field == "electrodes":
                        self._data[data_field] = [data.tactiles[i].electrodes]

    def timerEvent(self):  # pylint: disable=C0103
        for i in range(self._electrodes_to_display_count):
            try:
                self._tactile_point_widget[i].update_data(self._data[self._datatype_to_display][i])
            except IndexError:
                pass
        self._data_bar.update_values(self._data)
        self._data_bar.refresh()

    def get_datatype_to_display(self):
        return self._datatype_to_display

    def change_datatype_to_display(self, datatype):
        if self._succeded_config_load:
            if datatype in ["pac", "electrodes"]:
                self._datatype_to_display = datatype
                self._electrodes_to_display_count = len(self._data[self._datatype_to_display])

            for i in range(len(self._tactile_point_widget)):
                if i < self._electrodes_to_display_count:
                    self._tactile_point_widget[i].show()
                else:
                    self._tactile_point_widget[i].hide()


class FingerWidgetVisualBiotacBlank(FingerWidget):
    def __init__(self, finger, parent):
        super().__init__(None, finger, parent)
        layout = QHBoxLayout()
        no_tactile_label = QLabel()
        no_tactile_label.setText("No tactile sensor")
        layout.addWidget(no_tactile_label, alignment=Qt.AlignCenter)
        self.setLayout(layout)

    def start_timer_and_subscriber(self):
        pass

    def stop_timer_and_subscriber(self):
        pass
