#!/usr/bin/env python
#
# Copyright 2018 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#


import rospy
import rospkg


from qt_gui.plugin import Plugin
import QtCore
from QtCore import Qt, QEvent, QObject, pyqtSignal
import QtGui
from QtGui import *
import QtWidgets
from QtWidgets import *
from math import degrees
from copy import deepcopy

from cyberglove_calibrer import *
from cyberglove_mapper import *

from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

import sip

rootPath = os.path.join(
    rospkg.RosPack().get_path('sr_gui_cyberglove_calibrator'))


class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class QVLine(QFrame):
    def __init__(self):
        super(QVLine, self).__init__()
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)


class SrGuiCyberglovePointTweaker(QtWidgets.QWidget):
    """
    Adjustment for one calibration point
    """
    def _get_button(self, text, amount):
        button = QPushButton(text)
        button.clicked.connect(lambda: self._change_raw(amount))
        button.setMaximumSize(40, 40)
        return button

    def __init__(self, index, raw, calibrated, button_callback=lambda x, y: x):
        super(SrGuiCyberglovePointTweaker, self).__init__()
        self.setLayout(QGridLayout())
        self.layout().setVerticalSpacing(25)

        self._raw_value_label = QLabel()
        self._calibrated_value_label = QLabel()

        up1 = self._get_button("+++", 0.1)
        up2 = self._get_button("++", 0.01)
        up3 = self._get_button("+", 0.001)

        down1 = self._get_button("- - -", -0.1)
        down2 = self._get_button("- -", -0.01)
        down3 = self._get_button("-", -0.001)

        self.layout().addWidget(self._calibrated_value_label, 0, 0)
        self.layout().addWidget(self._raw_value_label, 1, 0)

        self.layout().addWidget(up1, 0, 1)
        self.layout().addWidget(up2, 0, 2)
        self.layout().addWidget(up3, 0, 3)
        self.layout().addWidget(down1, 1, 1)
        self.layout().addWidget(down2, 1, 2)
        self.layout().addWidget(down3, 1, 3)

        self._button_callback = button_callback

        self._set_raw_value(raw)
        self._set_calibrated_value(calibrated)
        self._index = index

    def _change_raw(self, amount):
        self._set_raw_value(self._raw_value + amount)
        self._button_callback(self._index, self._raw_value)

    def _set_calibrated_value(self, value):
        self._calibrated_value = value
        self._calibrated_value_label.setText("%.4f (Cal)" % self._calibrated_value)

    def _set_raw_value(self, value):
        self._raw_value = value
        self._raw_value_label.setText("%.4f (Raw)" % self._raw_value)


class SrGuiCybergloveJointTweaker(QtWidgets.QWidget):
    """
    Calibrator for one joint, expecting linear calibrations between two or more points.
    """
    # You can't set widget properties from outside main thread, so create a signal to updat with new sensor readings
    update_gui = pyqtSignal()

    def __init__(self, joint_name, calibration_changed_callback, calibration_points, tweak_callback):
        super(SrGuiCybergloveJointTweaker, self).__init__()

        self._tweak_callback = tweak_callback
        self._joint_name = joint_name

        self._calibration_points = calibration_points
        self._calibration_points.sort(key=lambda p: p[1])

        self.setLayout(QHBoxLayout())
        self.layout().addWidget(self._get_labels(joint_name))

        self.layout().addWidget(self._get_points_and_display(self._calibration_points))

        self.layout().addWidget(self._get_picture(joint_name))
        self.layout().addStretch()

        self.update_gui.connect(self._update_gui)

        # Updating Widgets too fast causes the gui to crash. Update everything at 20hz,
        # rather than each time a new value is ready.
        self._timer = rospy.Timer(rospy.Duration(0.05), lambda x: self.update_gui.emit())

    def _get_picture(self, joint_name):
        picture = QLabel()
        picture.setPixmap(QPixmap(rootPath + '/images/%s.jpg' % joint_name))
        return picture

    def _get_labels(self, joint_name):
        labels = QtWidgets.QWidget()
        labels.setLayout(QVBoxLayout())
        labels.layout().addWidget(QLabel(joint_name))

        self._raw_value_label = QLabel()
        self._calibrated_value_label = QLabel()

        labels.layout().addWidget(self._calibrated_value_label)
        labels.layout().addWidget(self._raw_value_label)
        return labels

    def _get_points_and_display(self, calibration_points):

        points = QtWidgets.QWidget()
        points.setLayout(QHBoxLayout())

        points_and_display = QtWidgets.QWidget()
        points_and_display.setLayout(QVBoxLayout())
        points_and_display.layout().setSpacing(30)

        for n, point in enumerate(calibration_points):
            points.layout().addWidget(SrGuiCyberglovePointTweaker(n, point[0], point[1], self._point_change_callback))
            if n < len(calibration_points) - 1:
                points.layout().addWidget(QVLine())

        self._progress_bar = QProgressBar()
        self._progress_bar.setTextVisible(False)
        self._progress_bar.setRange(0, 1000)

        points_and_display.layout().addWidget(self._progress_bar)
        points_and_display.layout().addWidget(points)

        return points_and_display

    def set_calibrated_value(self, value):
        self._calibrated_value = value

    def set_raw_value(self, value):
        self._raw_value = value

    def _set_progress_bar(self):
        """
        Split the sensor range into (number of cal points - 1) windows. Treat each window as a separate linear
        interpolation, so progress bar shows which calibration points are controlling the current value.
        """

        min_calibration = self._calibration_points[0][1]
        max_calibration = self._calibration_points[-1][1]

        position = max(self._calibrated_value, min_calibration)
        position = min(position, max_calibration)

        window_index = 0
        number_of_windows = (len(self._calibration_points) - 1)

        range_per_window = (self._progress_bar.maximum() - self._progress_bar.minimum()) / number_of_windows

        lower_bound = min_calibration
        upper_bound = max_calibration

        # Find which window the sensor value is in. Set upper and lower bounds to the edges the  window.
        for n in range(len(self._calibration_points) - 1):
            if position > self._calibration_points[n + 1][1]:
                lower_bound = self._calibration_points[n + 1][1]
                window_index += 1
            else:
                upper_bound = self._calibration_points[n + 1][1]
                break

        position_in_window = (position - lower_bound)/(upper_bound - lower_bound)
        value = (window_index + position_in_window) * range_per_window

        self._progress_bar.setValue(value)

    @QtCore.pyqtSlot()
    def _update_gui(self):
        try:
            self._calibrated_value_label.setText("%10.4f (Calibrated)" % self._calibrated_value)
            self._raw_value_label.setText("%10.4f (Raw)" % self._raw_value)
            self._set_progress_bar()
        except RuntimeError as e:
            # When the gui is closed, different threads die at different times. Sometimes an attempt is
            # made to write to set the values of a deleted label. This kills the unwanted error messages.
            if "has been deleted" not in str(e):
                rospy.logerr("Runtime error while updating gui: %s" % str(e))

    def _point_change_callback(self, index, value):
        self._calibration_points[index][0] = value
        self._tweak_callback(self._joint_name, self._calibration_points)


class SrGuiCybergloveTweaker(Plugin):
    """
    The plugin used to tweak glove calibrations.
    """
    name = "Cyberglove Calibration Tweaker"

    def _make_listeners(self):
        self._raw_listner = rospy.Subscriber("/rh_cyberglove/raw/joint_states",
                                             JointState, self._raw_callback, queue_size=1)
        self._calibrated_listner = rospy.Subscriber("/rh_cyberglove/calibrated/joint_states",
                                                    JointState, self._calibrated_callback, queue_size=1)

    def _get_calibration_from_parameter(self, calibration=None):
        if calibration is None:
            calibration = rospy.get_param(self._calibration_param)
        self._original_calibration = deepcopy(calibration)
        self._calibration = {}
        for entry in calibration:
            name = entry[0]
            self._calibration[name] = entry[1]

    def _raw_callback(self, msg):
        for n, joint in enumerate(msg.name):
            if joint in self._joint_tweakers:
                value = msg.position[n]
                self._joint_tweakers[joint].set_raw_value(value)

    def _calibrated_callback(self, msg):
        for n, joint in enumerate(msg.name):
            if joint in self._joint_tweakers:
                value = msg.position[n]
                self._joint_tweakers[joint].set_calibrated_value(degrees(value))

    def _tweak_callback(self, joint, calibration):
        self._calibration[joint] = calibration
        self._output_calibration_to_parameter()

    def _output_calibration_to_parameter(self, calibration=None):
        if calibration is None:
            calibration = [[name, self._calibration[name]] for name in self._joint_names]
        rospy.set_param(self._calibration_param, calibration)
        self._driver_reload_callback()

    def _make_widget(self, context):
        self._widget = QtWidgets.QWidget()

        if context is not None:
            context.add_widget(self._widget)

        self._top_layout = QVBoxLayout()
        self._joints_layout = QVBoxLayout()
        self._widget.setLayout(self._top_layout)

        top = QtWidgets.QWidget()
        top.setLayout(self._joints_layout)

        scroll = QScrollArea()
        scroll.setWidget(top)
        scroll.setWidgetResizable(True)

        self._top_layout.addWidget(scroll)

        self._top_layout.addWidget(QtWidgets.QWidget())

        self._get_calibration_from_parameter()

        self._add_tweakers_to_widget()

        self._make_buttons()

    def _add_tweakers_to_widget(self):
        self._joint_tweakers = {}
        self._joint_lines = {}

        for joint in self._joint_names:
            joint_tweaker = SrGuiCybergloveJointTweaker(joint, None, self._calibration[joint], self._tweak_callback)
            joint_tweaker.set_raw_value(0)
            joint_tweaker.set_calibrated_value(0)
            self._joint_tweakers[joint] = joint_tweaker
            self._joints_layout.addWidget(joint_tweaker)
            self._joint_lines[joint] = QHLine()
            self._joints_layout.addWidget(self._joint_lines[joint])

    def _make_calibrer(self):
        # read nb_sensors from rosparam or fallback to 22
        if rospy.has_param('rh_cyberglove/cyberglove_joint_number'):
            self.nb_sensors = rospy.get_param('rh_cyberglove/cyberglove_joint_number')
        else:
            self.nb_sensors = 22
        self._calibrer = CybergloveCalibrer(description_function=None, nb_sensors=self.nb_sensors)
        self._joint_names = self._calibrer.cyberglove.joints.keys()
        self._joint_names.sort()

    def __init__(self, context):
        super(SrGuiCybergloveTweaker, self).__init__(context)
        self.setObjectName('SrGuiCybergloveTweaker')
        self.icon_dir = os.path.join(
            rospkg.RosPack().get_path('sr_visualization_icons'), '/icons')

        self._calibration_param = "/rh_cyberglove/cyberglove_calibration"

        self._make_calibrer()

        self._make_widget(context)

        self._make_listeners()

        self._widget.setWindowTitle('Cyberglove Calibration Tweaker')

        self._driver_reload_callback = rospy.ServiceProxy("/rh_cyberglove/reload_calibration", Empty)

    def _make_buttons(self):
        btn_frame = QtWidgets.QFrame()
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.setSpacing(25)

        btn_save = QtWidgets.QPushButton()
        btn_save.setText("&Save")
        btn_save.setToolTip("Save the current calibration")
        btn_save.setIcon(QtGui.QIcon(rootPath + '/images/icons/save.png'))
        btn_save.clicked.connect(self._save_calibration)
        btn_layout.addWidget(btn_save)

        btn_load = QtWidgets.QPushButton()
        btn_load.setText("&Load")
        btn_load.setToolTip("Load a Glove calibration")
        btn_load.setIcon(QtGui.QIcon(rootPath + '/images/icons/load.png'))
        btn_layout.addWidget(btn_load)
        btn_load.clicked.connect(self._load_calibration)
        btn_frame.setLayout(btn_layout)

        btn_reset = QtWidgets.QPushButton()
        btn_reset.setText("&Reset")
        btn_reset.setToolTip("Reset to original/last loaded calibration.")
        btn_reset.setIcon(QtGui.QIcon(rootPath + '/images/icons/load.png'))
        btn_layout.addWidget(btn_reset)
        btn_reset.clicked.connect(self._reset_calibration)
        btn_frame.setLayout(btn_layout)

        self._top_layout.addWidget(btn_frame)

    def _delete_widget(self, layout, widget):
        layout.removeWidget(widget)
        sip.delete(widget)
        widget = None

    def _reset_calibration(self):
        self._output_calibration_to_parameter(self._original_calibration)
        self._get_calibration_from_parameter()
        self._reset_tweakers()

    def _reset_tweakers(self):
        for joint_name in self._joint_tweakers:
            self._delete_widget(self._joints_layout,  self._joint_tweakers[joint_name])
            self._delete_widget(self._joints_layout,  self._joint_lines[joint_name])
        self._add_tweakers_to_widget()

    def _save_calibration(self):
        (file_name, dummy) = QtWidgets.QFileDialog.getSaveFileName(
            self._widget, 'Save Calibration', '')
        if file_name == "":
            return

        text = ["{'cyberglove_calibration': ["]

        for name in self._joint_names:
            calibration_string = str(self._calibration[name])
            text.append("['%s', %s]," % (name, calibration_string))
        text.append("]}")

        try:
            with open(file_name, "w") as output_file:
                for line in text:
                    output_file.write(line + "\n")
        except Exception as e:
            rospy.logerr(str(e))
            QtWidgets.QMessageBox.information(
                self._widget, "Calibration saving failed!", "Saving loading failed: %s" % str(e),
                QtWidgets.QMessageBox.Ok, QtWidgets.QMessageBox.Ok)

    def _load_calibration(self):
        (file_name, dummy) = QtWidgets.QFileDialog.getOpenFileName(
            self._widget, 'Open Calibration', '')
        if "" == file_name:
            return

        if self._calibrer.load_calib(str(file_name)) == 0:
            QtWidgets.QMessageBox.information(
                self._widget, "Calibration successfully loaded", "Calibration successfully loaded.",
                QtWidgets.QMessageBox.Ok, QtWidgets.QMessageBox.Ok)
            # cyberglove_calibrer already had a function for loading from a file which writes to the parameter.
            # as we have to set the parameter anyway, we just let calibrer do it and read it back from the param server
            # it's a little bit dirty, but sue me, it works ;)

            self._get_calibration_from_parameter()
            self._reset_tweakers()

        else:
            QtWidgets.QMessageBox.information(
                self._widget, "Calibration loading failed", "Calibration loading failed",
                QtWidgets.QMessageBox.Ok, QtWidgets.QMessageBox.Ok)

if __name__ == '__main__':
    import sys
    rospy.init_node("test_gui")
    app = QApplication(sys.argv)
    window = SrGuiCybergloveTweaker(None)
    window._widget.show()
    sys.exit(app.exec_())
