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

rootPath = os.path.join(
    rospkg.RosPack().get_path('sr_gui_cyberglove_calibrator'))

class SrGuiCybergloveTweaker(Plugin):

    """
    The plugin used to tweak glove calibrations.
    """
    name = "Cyberglove Calibration Tweaker"

    def __init__(self, context):
        super(SrGuiCybergloveTweaker, self).__init__(context)
        self.setObjectName('SrGuiCybergloveTweaker')
        self.icon_dir = os.path.join(
            rospkg.RosPack().get_path('sr_visualization_icons'), '/icons')

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_cyberglove_calibrator'), 'uis', 'SrGuiCybergloveTweaker.ui')
        self._widget = QtWidgets.QWidget()
        loadUi(ui_file, self._widget)
        context.add_widget(self._widget)

        # self.frame = QtWidgets.QFrame()
        # self.layout = QtWidgets.QVBoxLayout()
        # self.frame.setLayout(self.layout)
        # self.window.setWidget(self.frame)

        # read nb_sensors from rosparam or fallback to 22
        if rospy.has_param('cyberglove/cyberglove_joint_number'):
            self.nb_sensors = rospy.get_param('cyberglove/cyberglove_joint_number')
        else:
            self.nb_sensors = 22

        self._calibrer = CybergloveCalibrer(description_function=None, nb_sensors=self.nb_sensors)
        self.joint_names = self._calibrer.cyberglove.joints.keys()
        self.joint_names.sort()




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
