#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
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

import sys
import rospy
import unittest
import rostest
from StringIO import StringIO
from mock import patch

from sr_gui_cyberglove_calibrator.cyberglove_calibrer import CybergloveCalibrer

NAME = "test_cyberglove_calibrator"
PKG = "sr_gui_cyberglove_calibrator"


class TestCybergloveCalibrator(unittest.TestCase):

    def setUp(self):
        self.held, sys.stdout = sys.stdout, StringIO()

    @patch('sr_gui_cyberglove_calibrator.cyberglove_calibrer.Cyberglove')
    def test_warning_message(self, Cyberglove):
        self.calibrer = CybergloveCalibrer(description_function=None)
        self.calibrer.load_calib("mock_file_name")
        self.assertEqual(sys.stdout.getvalue().strip(), 'Service not found, is the driver running?')

if __name__ == "__main__":
    rospy.init_node("test_cyberglove_calibrator")
    rostest.rosrun(PKG, NAME, TestCybergloveCalibrator)
