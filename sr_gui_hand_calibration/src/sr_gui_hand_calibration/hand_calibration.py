#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
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

import os

import roslib
roslib.load_manifest('sr_gui_hand_calibration')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QWidget, QShortcut, QColor, QTreeWidgetItem, QFileDialog, QMessageBox
from QtCore import QVariant
from sr_gui_hand_calibration.sr_hand_calibration_model import HandCalibration

class SrHandCalibration(Plugin):

    def __init__(self, context):
        super(SrHandCalibration, self).__init__(context)
        self.setObjectName('SrHandCalibration')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../uis/SrHandCalibration.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrHandCalibrationUi')
        context.add_widget(self._widget)

        self._widget.tree_calibration.setColumnCount(4)
        self._widget.tree_calibration.setHeaderLabels(["Finger", "Joint", "Raw Value", "Calibrated Value"])

        self.hand_model = None

        self._widget.btn_save.clicked.connect(self.btn_save_clicked_)
        self._widget.btn_load.clicked.connect(self.btn_load_clicked_)
        self._widget.btn_joint_0s.clicked.connect(self.btn_joint_0s_clicked_)

        self.populate_tree()

    def populate_tree(self):
        self._widget.tree_calibration.clear()

        self.hand_model = HandCalibration( tree_widget = self._widget.tree_calibration, progress_bar = self._widget.progress )
        if not self.hand_model.is_active:
            self.close_plugin()
            return

        self._widget.tree_calibration.expandAll()

        for col in range(0, self._widget.tree_calibration.columnCount()):
            self._widget.tree_calibration.resizeColumnToContents(col)

    def btn_save_clicked_(self):
        path_to_config = "~"
        try:
            path_to_config = roslib.packages.get_pkg_dir("sr_ethercat_hand_config") + "/calibrations"
        except:
            rospy.logwarn("couldnt find the sr_ethercat_hand_config package")

        filter_files = "*.yaml"
        filename, _ = QFileDialog.getOpenFileName(self._widget.tree_calibration, self._widget.tr('Save Calibration'),
                                                  self._widget.tr(path_to_config),
                                                  self._widget.tr(filter_files))

        if filename == "":
            return

        if not self.hand_model.is_calibration_complete():
            btn_pressed = QMessageBox.warning(self._widget.tree_calibration, "Warning", "Are you sure you want to save this incomplete calibration? The uncalibrated values will be saved as a flat map (the calibrated value will always be 0)",
                                              buttons = QMessageBox.Ok |  QMessageBox.Cancel)

            if btn_pressed == QMessageBox.Cancel:
                return
        self.hand_model.save( filename )

    def btn_load_clicked_(self):
        path_to_config = "~"
        try:
            path_to_config = roslib.packages.get_pkg_dir("sr_ethercat_hand_config") + "/calibrations"
        except:
            rospy.logwarn("couldnt find the sr_ethercat_hand_config package")

        filter_files = "*.yaml"
        filename, _ = QFileDialog.getOpenFileName(self._widget.tree_calibration, self._widget.tr('Load Calibration'),
                                                  self._widget.tr(path_to_config),
                                                  self._widget.tr(filter_files))

        if filename == "":
            return

        self.hand_model.load( filename )

    def btn_joint_0s_clicked_(self):
        self.hand_model.calibrate_joint0s( self._widget.btn_joint_0s )

    def _unregisterPublisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

        self.hand_model.unregister()

    def shutdown_plugin(self):
        self._unregisterPublisher()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass

