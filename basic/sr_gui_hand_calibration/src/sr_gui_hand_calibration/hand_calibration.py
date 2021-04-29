#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
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
#

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from PyQt5.uic import loadUi

from PyQt5.QtCore import QEvent, QObject, Qt, QTimer, Slot
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QWidget, QShortcut, QTreeWidgetItem, QFileDialog, QMessageBox
from PyQt5.QtCore import QVariant
from sr_gui_hand_calibration.sr_hand_calibration_model import HandCalibration


class SrHandCalibration(Plugin):

    """
    A rosgui plugin for calibrating the Shadow EtherCAT Hand
    """

    def __init__(self, context):
        super(SrHandCalibration, self).__init__(context)
        self.setObjectName('SrHandCalibration')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_hand_calibration'), 'uis', 'SrHandCalibration.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrHandCalibrationUi')
        context.add_widget(self._widget)

        self._widget.tree_calibration.setColumnCount(4)
        self._widget.tree_calibration.setHeaderLabels(
            ["Finger", "Joint", "Raw Value", "Calibrated Value"])

        self.hand_model = None

        self._widget.btn_save.clicked.connect(self.btn_save_clicked_)
        self._widget.btn_load.clicked.connect(self.btn_load_clicked_)
        self._widget.cb_old_version.stateChanged.connect(self.cb_state_changed_)
        self._widget.btn_joint_0s.clicked.connect(self.btn_joint_0s_clicked_)

        self.populate_tree()

    def populate_tree(self, old_version=False):
        """
        Create tree with calibrations
        """
        self._widget.tree_calibration.clear()

        self.hand_model = HandCalibration(
            tree_widget=self._widget.tree_calibration, progress_bar=self._widget.progress, old_version=old_version)
        if not self.hand_model.is_active:
            self.close_plugin()
            return

        self._widget.tree_calibration.expandAll()

        for col in range(0, self._widget.tree_calibration.columnCount()):
            self._widget.tree_calibration.resizeColumnToContents(col)

    def btn_save_clicked_(self):
        """
        Save calibration to a yaml file.
        sr_ethercat_hand_config package must be installed
        """
        path_to_config = "~"
        # Reading the param that contains the config_dir suffix that we should
        # use for this hand (e.g. '' normally for a right hand  or 'lh' if this
        # is for a left hand)
        config_dir = rospy.get_param('config_dir', '')
        try:
            path_to_config = os.path.join(rospkg.RosPack().get_path(
                'sr_ethercat_hand_config'), 'calibrations', config_dir)
        except Exception:
            rospy.logwarn("couldnt find the sr_ethercat_hand_config package")

        filter_files = "*.yaml"
        filename, _ = QFileDialog.getOpenFileName(
            self._widget.tree_calibration, self._widget.tr('Save Calibration'),
            self._widget.tr(
                path_to_config),
            self._widget.tr(filter_files))

        if filename == "":
            return

        if not self.hand_model.is_calibration_complete():
            btn_pressed = QMessageBox.warning(
                self._widget.tree_calibration, "Warning", "Are you sure you want to save this incomplete calibration?"
                " The uncalibrated values will be saved as a flat map (the calibrated value will always be 0)",
                buttons=QMessageBox.Ok | QMessageBox.Cancel)

            if btn_pressed == QMessageBox.Cancel:
                return
        self.hand_model.save(filename)

    def btn_load_clicked_(self):
        """
        Load calibration from a yaml file.
        sr_ethercat_hand_config package must be installed
        """
        path_to_config = "~"
        # Reading the param that contains the config_dir suffix that we should
        # use for this hand (e.g. '' normally for a right hand  or 'lh' if this
        # is for a left hand)
        config_dir = rospy.get_param('config_dir', '')
        try:
            path_to_config = os.path.join(rospkg.RosPack().get_path('sr_ethercat_hand_config'),
                                          'calibrations', config_dir)
        except Exception:
            rospy.logwarn("couldn't find the sr_ethercat_hand_config package")

        filter_files = "*.yaml"
        filename, _ = QFileDialog.getOpenFileName(self._widget.tree_calibration,
                                                  self._widget.tr('Load Calibration'),
                                                  self._widget.tr(path_to_config),
                                                  self._widget.tr(filter_files))

        if filename == "":
            return

        self.hand_model.load(filename)

    def btn_joint_0s_clicked_(self):
        """
        calibrate the first joint of each finger
        """
        self.hand_model.calibrate_joint0s(self._widget.btn_joint_0s)

    def cb_state_changed_(self):
        if self._widget.cb_old_version.isChecked():
            self.populate_tree(old_version=True)
        else:
            self.populate_tree(old_version=False)

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
