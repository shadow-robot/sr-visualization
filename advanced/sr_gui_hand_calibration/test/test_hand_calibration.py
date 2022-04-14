#!/usr/bin/env python3

# Copyright 2020, 2022 Shadow Robot Company Ltd.
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

# This file contains an alternate approach to pedal connection, that falls victim to Ubuntu suspending USB devices.
# Try as I might, I couldn't make Ubuntu leave it alone, so switched to the input events approach seen in
# sr_triple_pedal.py. I'll leave this here in case it might be useful later.

from __future__ import absolute_import
import sys
import os
import tempfile
import unittest
import rospy
import rospkg
import rostest
from mock import patch
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QWidget, QApplication
from pyvirtualdisplay import Display
from sr_gui_hand_calibration.sr_hand_calibration_model import HandCalibration

NAME = "test_hand_calibration"
PKG = "sr_gui_hand_calibration"


class TestHandCalibration(unittest.TestCase):

    def setUp(self):
        display = Display(visible=False, size=(1024, 768), color_depth=24)
        display.start()
        self.hand_model = None
        self.app = QApplication(sys.argv)
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_hand_calibration'), 'uis', 'SrHandCalibration.ui')
        loadUi(ui_file, self._widget)

        self.mock_file = tempfile.NamedTemporaryFile(delete=False)
        self.mock_file.write(b"""sr_calibrations: [\n""" +
                             b"""["mock", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]],\n""" +
                             b"""]""")
        self.mock_file.write(b"""\n\nsr_calibrations_coupled: [\n""" +
                             b"""[["mock_1", "mock_2"], [[[0.0, 0.0], 0.0, 0.0]]],\n""" +
                             b"""]""")
        self.mock_file.close()

    def tearDown(self):
        os.remove(self.mock_file.name)

    @patch('sr_gui_hand_calibration.sr_hand_calibration_model.EtherCAT_Hand_Lib')
    def test_progress_bar(self, _EtherCAT_Hand_Lib):
        self.hand_model = HandCalibration(tree_widget=self._widget.tree_calibration,
                                          progress_bar=self._widget.progress)

        self.assertEqual(self.hand_model.progress_bar.value(), 0)
        self.hand_model.load(self.mock_file.name)
        self.assertEqual(self.hand_model.progress_bar.value(), 100)


if __name__ == "__main__":
    rospy.init_node("test_hand_calibration")
    rostest.rosrun(PKG, NAME, TestHandCalibration)
