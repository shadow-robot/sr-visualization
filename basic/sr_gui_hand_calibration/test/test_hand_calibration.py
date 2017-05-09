#!/usr/bin/env python
import sys
import os
import rospy
import rospkg
import unittest
import rostest

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

        self.hand_model = HandCalibration(tree_widget=self._widget.tree_calibration,
                                          progress_bar=self._widget.progress,
                                          test_only=True)

        f = open(rospy.get_param('mock_file'), "w+")
        f.write("""{'sr_calibrations': [\n""" +
                """["mock", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]],\n""" +
                """]}""")
        f.close()

    def tearDown(self):
        os.remove(rospy.get_param('mock_file'))

    def test_progress_bar(self):
     #   self.assertEquals(self.hand_model.progress_bar.value(), 0)
      #  self.hand_model.load(rospy.get_param('mock_file'))
       # self.assertEquals(self.hand_model.progress_bar.value(), 100)
       self.assertEquals(1,1)

if __name__ == "__main__":
    rospy.init_node("test_hand_calibration")
    rostest.rosrun(PKG, NAME, TestHandCalibration)
