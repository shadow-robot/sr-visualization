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
from python_qt_binding import loadUi
import QtCore
from QtCore import Qt, QEvent, QObject
import QtGui
from QtGui import *
import QtWidgets
from QtWidgets import *

from cyberglove_calibrer import *
from cyberglove_mapper import *

from sensor_msgs.msg import JointState

rootPath = os.path.join(
    rospkg.RosPack().get_path('sr_gui_cyberglove_calibrator'))

class SrGuiCyberglovePointTweaker(QtWidgets.QWidget):
    """
    Adjustment for one calibration point
    """
    def _get_button(self, text, amount):
        button = QPushButton(text)
        button.clicked.connect(lambda: self._change_raw(amount))
        button.setMaximumSize(40,40)
        return button

    def __init__(self, index, raw, calibrated, increment=0.01, button_callback=lambda x: x):
        super(SrGuiCyberglovePointTweaker, self).__init__()
        self.setLayout(QGridLayout())

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
        self._increment = increment

        self._set_raw_value(raw)
        self._set_calibrated_value(calibrated)
        self._index = index

    def _change_raw(self, amount):
        self._set_raw_value(self._raw_value + amount)
        self._button_callback(self._raw_value)

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

    def __init__(self, joint_name, calibration_changed_callback, calibration_points=[]):
        super(SrGuiCybergloveJointTweaker, self).__init__()

        self.setLayout(QHBoxLayout())

        labels = QtWidgets.QWidget()
        labels.setLayout(QVBoxLayout())
        labels.layout().addWidget(QLabel(joint_name))

        self._raw_value_label = QLabel()
        self._calibrated_value_label = QLabel()

        labels.layout().addWidget(self._calibrated_value_label)
        labels.layout().addWidget(self._raw_value_label)

        self._calibration_points = calibration_points
        self._calibration_points.sort(key = lambda p: p[1])

        self.layout().addWidget(labels)
        for n,point in enumerate(calibration_points):
            self.layout().addWidget(SrGuiCyberglovePointTweaker(n, point[0], point[1]))
        self.layout().addStretch()

        self._timer = rospy.Timer(rospy.Duration(0.1), self._update)

    def set_calibrated_value(self, value):
        self._calibrated_value = value

    def set_raw_value(self, value):
        self._raw_value = value

    def _update(self, event):
        self._calibrated_value_label.setText("%10.4f (Calibrated)" % self._calibrated_value)
        self._raw_value_label.setText("%10.4f (Raw)" % self._raw_value)

class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)

# class CalibrationPoint

# class JointCalibration(object)

# class JointCalibration(object):
#     def __init__(self, specification):
#         if (type(specification) != list or len(specification) != 2 or
#             type(specification[0]) != str or type(specification[1]) != list):
#             rospy.logfatal("Incorrectly specified calibration point: %s" % str(specification))
#         self._name = specification[0]

#         raw = []
#         cal = []

#         for entry in specification[1]:
#             if type(entry) != list or len(entry) != 2:
#                 rospy.logfatal("Incorrectly specified calibration point: %s" % str(specification))
#             sel

class SrGuiCybergloveTweaker(Plugin):

    """
    The plugin used to tweak glove calibrations.
    """
    name = "Cyberglove Calibration Tweaker"

    def _make_listeners(self):
        self._raw_listner  = rospy.Subscriber("/rh_cyberglove/raw/joint_states",
                                              JointState, self._raw_callback, queue_size=1)
        self._calibrated_listner = rospy.Subscriber("/rh_cyberglove/calibrated/joint_states",
                                                    JointState, self._calibrated_callback, queue_size=1)

    def _raw_callback(self, msg):
        for n, joint in enumerate(msg.name):
            value = msg.position[n]
            self._joint_tweakers[joint].set_raw_value(value)

    def _calibrated_callback(self, msg):
        for n, joint in enumerate(msg.name):
            value = msg.position[n]
            self._joint_tweakers[joint].set_calibrated_value(value)

    def _make_widget(self, context):
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_cyberglove_calibrator'), 'uis', 'SrGuiCybergloveTweaker.ui')
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

        self._joint_tweakers = {}

        for joint in self._joint_names:
            joint_tweaker = SrGuiCybergloveJointTweaker(joint, None, [[0.02,50.0],[0.55,0.0]])
            joint_tweaker.set_raw_value(0)
            joint_tweaker.set_calibrated_value(0)
            self._joint_tweakers[joint] = joint_tweaker
            self._joints_layout.addWidget(joint_tweaker)
            self._joints_layout.addWidget(QHLine())

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


        self._make_calibrer()

        self._make_widget(context)

        self._make_listeners()

        # self.layout = self._widget.layout
        # subframe = QtWidgets.QFrame()
        # sublayout = QtWidgets.QHBoxLayout()

        # self.glove_calibrating_widget = GloveCalibratingWidget(
        #     self._widget, self.joint_names)
        # self.layout.addWidget(self.glove_calibrating_widget)

        # self.step_selector = StepSelector(self._widget, self.calibrer)
        # sublayout.addWidget(self.step_selector)

        # btn_frame = QtWidgets.QFrame()
        # btn_layout = QtWidgets.QVBoxLayout()
        # btn_layout.setSpacing(25)
        # btn_calibrate = QtWidgets.QPushButton()
        # btn_calibrate.setText("Calibrate")
        # btn_calibrate.setToolTip("Calibrate the current selected step")
        # btn_calibrate.setIcon(
        #     QtGui.QIcon(rootPath + '/images/icons/calibrate.png'))
        # btn_layout.addWidget(btn_calibrate)
        # btn_calibrate.clicked.connect(self.calibrate_current_step)
        # self.btn_save = QtWidgets.QPushButton()
        # self.btn_save.setText("Save")
        # self.btn_save.setToolTip("Save the current calibration")
        # self.btn_save.setIcon(QtGui.QIcon(rootPath + '/images/icons/save.png'))
        # self.btn_save.setDisabled(True)
        # btn_layout.addWidget(self.btn_save)
        # self.btn_save.clicked.connect(self.save_calib)
        # btn_load = QtWidgets.QPushButton()
        # btn_load.setText("Load")
        # btn_load.setToolTip("Load a Glove calibration")
        # btn_load.setIcon(QtGui.QIcon(rootPath + '/images/icons/load.png'))
        # btn_layout.addWidget(btn_load)
        # btn_load.clicked.connect(self.load_calib)
        # btn_frame.setLayout(btn_layout)
        # sublayout.addWidget(btn_frame)
        # subframe.setLayout(sublayout)
        # self.layout.addWidget(subframe)

        # #  QtCore.QTimer.singleShot(0, self.window.adjustSize)

    # def calibrate_current_step(self):
    #     self.step_selector.calibrate_current_step()

    #     for name in self.joint_names:
    #         if self.calibrer.is_step_done(name) == 0.5:
    #             self.glove_calibrating_widget.set_half_calibrated([name])
    #         elif self.calibrer.is_step_done(name) == 1.0:
    #             self.glove_calibrating_widget.set_calibrated([name])

    #     if self.calibrer.all_steps_done():
    #         range_errors = self.calibrer.check_ranges()
    #         if len(range_errors) != 0:
    #             QtWidgets.QMessageBox.warning(self._widget, "%d ensor range error(s) reported." % len(range_errors),
    #                                           "\n".join(range_errors),
    #                                           QtWidgets.QMessageBox.Ok,
    #                                           QtWidgets.QMessageBox.Ok)

    #         self.btn_save.setEnabled(True)

    # def save_calib(self):
    #     # since pyqt5, filters are also returned
    #     (filename, dummy) = QtWidgets.QFileDialog.getSaveFileName(
    #         self._widget, 'Save Calibration', '')
    #     if filename == "":
    #         return

    #     write_output = self.calibrer.write_calibration_file(filename)
    #     error = None
    #     if write_output == -1:
    #         error = "Calibration has not been finished, output not written."
    #     elif write_output == -2:
    #         error = "Error writing file."

    #     if error is not None:
    #         QtWidgets.QMessageBox.error(self._widget, "Error writing config.",
    #                                     error,
    #                                     QtWidgets.QMessageBox.Cancel,
    #                                     QtWidgets.QMessageBox.Cancel)
    #     elif QtWidgets.QMessageBox.question(self._widget,
    #                                         "Load new Calibration",
    #                                         "Do you want to load the new calibration file?",
    #                                         QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
    #                                         QtWidgets.QMessageBox.No) == QtWidgets.QMessageBox.Yes:
    #         self.load_calib(filename)

    # def load_calib(self, filename=""):
    #     # when called from the button. filename is filled with some more info
    #     if filename is False:
    #         filename = ""
    #     if "" == filename:
    #         # since pyqt5, filters are also returned
    #         (filename, dummy) = QtWidgets.QFileDialog.getOpenFileName(
    #             self._widget, 'Open Calibration', '')
    #         if "" == filename:
    #             return

    #     if self.calibrer.load_calib(str(filename)) == 0:
    #         # statusbar do not exist in rqt anymore (since shared with several plugins)
    #         # self.statusBar().showMessage("New Cyberglove Calibration Loaded.")
    #         QtWidgets.QMessageBox.information(self._widget, "Calibration successfully loaded",
    #                                           "Calibration successfully loaded.",
    #                                           QtWidgets.QMessageBox.Ok,
    #                                           QtWidgets.QMessageBox.Ok)
    #     else:
    #         QtWidgets.QMessageBox.information(self._widget, "Calibration loading failed",
    #                                           "Calibration loading failed",
    #                                           QtWidgets.QMessageBox.Ok,
    #                                           QtWidgets.QMessageBox.Ok)

if __name__ == '__main__':
    import sys
    rospy.init_node("test_gui")
    app = QApplication(sys.argv)
    window = SrGuiCybergloveTweaker(None)
    window._widget.show()
    sys.exit(app.exec_())
