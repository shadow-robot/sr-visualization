#!/usr/bin/env python
import sys
import os
import rospy
import rospkg
import unittest
import rostest
import tempfile
from mock import patch

from sr_gui_hand_calibration.sr_hand_calibration_model import HandCalibration

from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QWidget, QApplication

NAME = "test_hand_calibration"
PKG = "sr_gui_hand_calibration"


class TestHandCalibration(unittest.TestCase):

    def setUp(self):
        self.app = QApplication(sys.argv)
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_hand_calibration'), 'uis', 'SrHandCalibration.ui')
        loadUi(ui_file, self._widget)

        self.mock_file = tempfile.NamedTemporaryFile(delete=False)
        self.mock_file.write("""{'sr_calibrations': [\n""" +
                             """["mock", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]],\n""" +
                             """]}""")
        self.mock_file.close()

    def tearDown(self):
        os.remove(self.mock_file.name)

    @patch('sr_gui_hand_calibration.sr_hand_calibration_model.EtherCAT_Hand_Lib')
    def test_progress_bar(self, EtherCAT_Hand_Lib):
        self.hand_model = HandCalibration(tree_widget=self._widget.tree_calibration,
                                          progress_bar=self._widget.progress)

        self.assertEquals(self.hand_model.progress_bar.value(), 0)
        self.hand_model.load(self.mock_file.name)
        self.assertEquals(self.hand_model.progress_bar.value(), 100)

if __name__ == "__main__":
    rospy.init_node("test_hand_calibration")
    rostest.rosrun(PKG, NAME, TestHandCalibration)
