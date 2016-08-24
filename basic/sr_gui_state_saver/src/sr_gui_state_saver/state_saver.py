#!/usr/bin/env python

# Disabling E1002 check since it complains about super for no reason -
# inheriting from QObject
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

import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import Qt, QThread, SIGNAL, QPoint
import QtCore
from QtGui import QWidget, QMessageBox, QFrame, \
    QHBoxLayout, QCheckBox, QLabel, QColor

from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticArray
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_robot_state_saver import SrStateSaverUnsafe


class SrGuiStateSaver(Plugin):

    """
    A gui plugin for resetting motors on the shadow hand.
    """

    def __init__(self, context):
        super(SrGuiStateSaver, self).__init__(context)
        self.setObjectName('SrGuiStateSaver')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(
            rospkg.RosPack().get_path('sr_gui_state_saver'), 'uis',
            'SrGuiStateSaver.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrStateSaverUi')
        context.add_widget(self._widget)

        QtCore.QObject.connect(self._widget.button_save, QtCore.SIGNAL("clicked()"), self._button_pressed)

    def _button_pressed(self):
        name = self._widget.edit_name.text()

        if name == "":
            QMessageBox.warning(self._widget, "No name entered!", "You must enter a name to save the state.")
            return

        which = ""
        if self._widget.radio_hand.isChecked():
            which = "hand"
        elif self._widget.radio_arm.isChecked():
            which = "arm"
        elif self._widget.radio_both.isChecked():
            which = "both"
        else:
            QMessageBox.warning(self._widget, "Choose what to save!",
                                "You must choose which part of the robot you are saving for.")
            return

        SrStateSaverUnsafe(name, which)
