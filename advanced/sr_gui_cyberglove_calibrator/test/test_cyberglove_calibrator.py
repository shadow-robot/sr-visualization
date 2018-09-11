#!/usr/bin/env python
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
<<<<<<< HEAD
        self.assertEqual(sys.stdout.getvalue().strip(), 'Srvice not found, is the driver running?')
=======
        self.assertEqual(sys.stdout.getvalue().strip(), 'Service not found, is the driver running?')
>>>>>>> F#SRC-2007_gui_update

if __name__ == "__main__":
    rospy.init_node("test_cyberglove_calibrator")
    rostest.rosrun(PKG, NAME, TestCybergloveCalibrator)
