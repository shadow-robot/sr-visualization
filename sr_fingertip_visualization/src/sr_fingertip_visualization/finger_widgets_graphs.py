#!/usr/bin/env python3

# Copyright 2022-2024 Shadow Robot Company Ltd.
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
import subprocess
import roslaunch
import rospy
import rospkg
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QPushButton,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QLabel,
    QCheckBox
)
from sr_robot_msgs.msg import ShadowPST, BiotacAll
from sr_fingertip_visualization.generic_plots import GenericDataPlot


class FingerWidgetGraphGeneric(QGroupBox):

    _BUFFER_SIZE = 350
    _CONST_FINGERS = ['ff', 'mf', 'rf', 'lf', 'th']

    def __init__(self, finger, side, parent):
        super().__init__(parent=parent)
        self._finger = finger
        self._side = side
        self._data = {}
        self._data_checkboxes = {}
        self._timer = QTimer()
        self._subscriber = None

        self.setTitle(finger)
        self.setCheckable(True)
        self.setChecked(False)
        self.clicked.connect(self.refresh_widget)

        icon_dir = os.path.join(rospkg.RosPack().get_path('sr_visualization_icons'), 'icons')
        self.plot_descriptors = {
            'blue': QIcon(os.path.join(icon_dir, 'blue.png')),
            'red': QIcon(os.path.join(icon_dir, 'red.png')),
            'green': QIcon(os.path.join(icon_dir, 'green.png')),
            'magenta': QIcon(os.path.join(icon_dir, 'magenta.png')),
            'gray': QIcon(os.path.join(icon_dir, 'gray.png')),
            'cyan': QIcon(os.path.join(icon_dir, 'cyan.png'))
        }

    def refresh_widget(self):
        if not self.isChecked():
            self.stop_timer_and_subscriber()
        else:
            self.start_timer_and_subscriber()

    def stop_timer_and_subscriber(self):
        self._timer.stop()
        if self._subscriber:
            self._subscriber.unregister()
            self._subscriber = None

    def get_data_checkboxes(self):
        return self._data_checkboxes


class FingerWidgetGraphPST(FingerWidgetGraphGeneric):

    _CONST_DATA_FIELDS = ['pressure', 'temperature']

    def __init__(self, side, finger, parent):
        super().__init__(finger, side, parent=parent)
        self._initialize_data_structure()

        self._tactile_data_callback(rospy.wait_for_message('/{}/tactile'.format(self._side), ShadowPST))
        self._plot_colors = list(self.plot_descriptors.keys())[:len(list(self._data.keys()))]
        self._plot = GenericDataPlot(self._data, list(self.plot_descriptors.keys()))

        plot_checkboxes = QGroupBox("Data")
        plot_checkboxes.setSizePolicy(1, 2)
        plot_checkboxes_layout = QHBoxLayout()
        for i, data_field in enumerate(self._CONST_DATA_FIELDS):
            self._data_checkboxes[data_field] = QCheckBox(data_field)
            self._data_checkboxes[data_field].setIcon(self.plot_descriptors[self._plot_colors[i]])
            self._data_checkboxes[data_field].stateChanged.connect(self.action_data_checkbox)
            plot_checkboxes_layout.addWidget(self._data_checkboxes[data_field])
        plot_checkboxes.setLayout(plot_checkboxes_layout)

        layout = QVBoxLayout()
        layout.addWidget(plot_checkboxes, alignment=Qt.AlignTop)
        layout.addWidget(self._plot)

        self.setLayout(layout)
        self.start_timer_and_subscriber()

    def _initialize_data_structure(self):
        for data_field in self._CONST_DATA_FIELDS:
            self._data[data_field] = [0]

    def action_data_checkbox(self, state):
        caller = self.sender()
        if isinstance(caller, QCheckBox):
            self._plot.show_trace(caller.text(), state)

    def start_timer_and_subscriber(self):
        if not self._subscriber:
            self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), ShadowPST,
                                                self._tactile_data_callback)
            self._timer.timeout.connect(self.timerEvent)
            self._timer.start(10)

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._CONST_FINGERS):
            if finger == self._finger:
                for data_field in self._CONST_DATA_FIELDS:
                    if len(self._data[data_field]) >= self._BUFFER_SIZE:
                        self._data[data_field] = self._data[data_field][1:]
                    if data_field == "pressure":
                        self._data[data_field].append(data.pressure[i])
                    elif data_field == "temperature":
                        self._data[data_field].append(data.temperature[i])

    def timerEvent(self):  # pylint: disable=C0103
        checked_data_fields = []
        for data_field in self._CONST_DATA_FIELDS:
            if self._data_checkboxes[data_field].isChecked():
                checked_data_fields.append(data_field)
                self._plot.update_plot(self._data)
        # Decide on the scaling depending if more than one checkbox is checked
        if len(checked_data_fields) > 1:
            self._plot.set_auto_scale()
        elif len(checked_data_fields) == 1:
            self._plot.set_trace_scale(checked_data_fields[0])


class FingerWidgetGraphBiotac(FingerWidgetGraphGeneric):

    _CONST_DATA_FIELDS = ['pac0', 'pac1', 'pdc', 'tac', 'tdc']

    def __init__(self, side, finger, parent):
        super().__init__(finger, side, parent=parent)
        self._initialize_data_structure()
        self._tactile_data_callback(rospy.wait_for_message('/{}/tactile'.format(self._side), BiotacAll))

        self._plot_colors = list(self.plot_descriptors.keys())[:len(list(self._data.keys()))]
        self._plot = GenericDataPlot(self._data, list(self.plot_descriptors.keys()))

        plot_checkboxes = QGroupBox("Data")
        plot_checkboxes.setSizePolicy(1, 2)
        plot_checkboxes_layout = QGridLayout()
        for i, data_field in enumerate(self._CONST_DATA_FIELDS):
            self._data_checkboxes[data_field] = QCheckBox(data_field)
            self._data_checkboxes[data_field].setIcon(self.plot_descriptors[self._plot_colors[i]])
            self._data_checkboxes[data_field].stateChanged.connect(self.action_data_checkbox)

        plot_checkboxes_layout.addWidget(self._data_checkboxes['pac0'], 0, 0, alignment=Qt.AlignLeft)
        plot_checkboxes_layout.addWidget(self._data_checkboxes['pac1'], 0, 1, alignment=Qt.AlignLeft)
        plot_checkboxes_layout.addWidget(self._data_checkboxes['pdc'], 0, 2, alignment=Qt.AlignLeft)
        plot_checkboxes_layout.addWidget(self._data_checkboxes['tac'], 1, 0, alignment=Qt.AlignLeft)
        plot_checkboxes_layout.addWidget(self._data_checkboxes['tdc'], 1, 1, alignment=Qt.AlignLeft)

        plot_checkboxes.setLayout(plot_checkboxes_layout)

        layout = QVBoxLayout()
        layout.addWidget(plot_checkboxes, alignment=Qt.AlignTop)
        layout.addWidget(self._plot)

        self.setLayout(layout)
        self.start_timer_and_subscriber()

    def _initialize_data_structure(self):
        for data_field in self._CONST_DATA_FIELDS:
            self._data[data_field] = []

    def action_data_checkbox(self, state):
        caller = self.sender()
        if isinstance(caller, QCheckBox):
            self._plot.show_trace(caller.text(), state)

    def _tactile_data_callback(self, data):
        for i, finger in enumerate(self._CONST_FINGERS):
            if finger == self._finger:
                for data_field in self._CONST_DATA_FIELDS:
                    if len(self._data[data_field]) >= self._BUFFER_SIZE:
                        self._data[data_field] = self._data[data_field][1:]
                    if data_field == "pac0":
                        self._data[data_field].append(data.tactiles[i].pac0)
                    elif data_field == "pac1":
                        self._data[data_field].append(data.tactiles[i].pac1)
                    elif data_field == "pdc":
                        self._data[data_field].append(data.tactiles[i].pdc)
                    elif data_field == "tac":
                        self._data[data_field].append(data.tactiles[i].tac)
                    elif data_field == "tdc":
                        self._data[data_field].append(data.tactiles[i].tdc)

    def start_timer_and_subscriber(self):
        if not self._subscriber:
            self._subscriber = rospy.Subscriber('/{}/tactile'.format(self._side), BiotacAll,
                                                self._tactile_data_callback)
            self._timer.timeout.connect(self.timerEvent)
            self._timer.start(10)

    def timerEvent(self):  # pylint: disable=C0103
        checked_data_fields = []
        for data_field in self._CONST_DATA_FIELDS:
            if self._data_checkboxes[data_field].isChecked():
                checked_data_fields.append(data_field)
                self._plot.update_plot(self._data)
        # Decide on the scaling depending if more than one checkbox is checked
        if len(checked_data_fields) > 1:
            self._plot.set_auto_scale()
        elif len(checked_data_fields) == 1:
            self._plot.set_trace_scale(checked_data_fields[0])


class FingerWidgetGraphBiotacBlank(FingerWidgetGraphGeneric):
    def __init__(self, side, finger, parent):
        super().__init__(finger, side, parent=parent)
        layout = QHBoxLayout()
        layout.addWidget(QLabel("No tactile sensor"))
        self.setLayout(layout)

    def start_timer_and_subscriber(self):
        pass

    def stop_timer_and_subscriber(self):
        pass


class FingerWidgetGraphPSTBlank(FingerWidgetGraphGeneric):
    def __init__(self, side, finger, parent):
        super().__init__(finger, side, parent=parent)
        layout = QHBoxLayout()
        layout.addWidget(QLabel("No tactile sensor"))
        self.setLayout(layout)

    def start_timer_and_subscriber(self):
        pass

    def stop_timer_and_subscriber(self):
        pass


class FingerWidgetGraphMSTBlank(QGroupBox):
    def __init__(self, side, parent):
        super().__init__(parent=parent)
        self._side = side

        self.setTitle("---")
        self.setCheckable(True)
        self.setChecked(True)
        self.setSizePolicy(1, 1)

        self.package = 'sr_mst'
        self.launch_file = 'sr_mst_hand_rviz_visualiser.launch'

        layout = QVBoxLayout()

        layout.setAlignment(Qt.AlignVCenter)
        no_tactile_label = QLabel("\t\tSTF sensors not supported yet."
                                  " \nAlternatively, you can launch the RViz or PlotJuggler visualization tools:")
        layout.addWidget(no_tactile_label, alignment=Qt.AlignCenter)

        self.launch_viz_button = QPushButton("Launch RViz")
        self.launch_viz_button.clicked.connect(self._button_action_launch_viz)
        layout.addWidget(self.launch_viz_button, alignment=Qt.AlignCenter)

        self.launch_plotjuggler_button = QPushButton("Launch PlotJuggler*")
        self.launch_plotjuggler_button.clicked.connect(self._button_action_launch_plotjuggler)
        layout.addWidget(self.launch_plotjuggler_button, alignment=Qt.AlignCenter)

        note_label = QLabel("*Note: You'll need to click 'Yes' to Start the Streaming and select"
                            f" /{self._side}/tactile data topic.")
        layout.addWidget(note_label, alignment=Qt.AlignCenter)

        self.setLayout(layout)

    def _button_action_launch_viz(self):
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # rospack = rospkg.RosPack()
        # # Specify launch file
        # path_to_launch_file = f"{rospack.get_path(self.package)}/launch/{self.executable}"

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Define launch arguments as a list of strings
        args = [self.package, self.launch_file, f'hand_id:={self._side}', 'publishing_frequency:=30']

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
        roslaunch_args = args[2:]
        
        launch_file = [(roslaunch_file, roslaunch_args)]

        # Launch file with arguments
        parent = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

        parent.start()
            

        # print process.is_alive()
        # process.stop()
        
        # command = f"roslaunch sr_mst sr_mst_hand_rviz_visualiser.launch hand_id:={self._side} publishing_frequency:=30"
        # rospy.loginfo(f"Launching RViz for visualization of STF fingertips: {command}")
        # subprocess.Popen(command, shell=True)  # pylint: disable=R1732
        self.setChecked(False)

    def _button_action_launch_plotjuggler(self):
        command = f"roslaunch sr_mst sr_mst_hand_plotjuggler_visualiser.launch hand_id:={self._side}"
        rospy.loginfo(f"Launching PlotJuggler for visualization of STF fingertips data: {command}")
        subprocess.Popen(command, shell=True)  # pylint: disable=R1732
        self.setChecked(False)

    def start_timer_and_subscriber(self):
        pass

    def stop_timer_and_subscriber(self):
        pass
