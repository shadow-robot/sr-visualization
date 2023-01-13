#!/usr/bin/env python3

# Copyright 2011, 2022-2023 Shadow Robot Company Ltd.
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

# pylint: disable=W0212


import sys

from unittest import TestCase
import threading
import yaml
import rostest
import rospkg
import rospy
from python_qt_binding.QtWidgets import QWidget, QApplication
from sr_health_check.sr_health_check_gui import SrHealthCheck
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck


class TestSrHealthCheck(TestCase):

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication(sys.argv)
        cls.sr_health_check = SrHealthCheck(None)
        cls.sr_health_check._results_file = f"{rospkg.RosPack().get_path('sr_health_check')}/test/results.yaml"
        with open(cls.sr_health_check._results_file, 'w', encoding="ASCII") as yaml_file:
            yaml.safe_dump({}, stream=yaml_file, default_flow_style=False)
        cls.sr_health_check._current_data = {}

    def test_initialize_checks(self):
        self.sr_health_check.initialize_checks()
        for check_name in self.sr_health_check._check_names:
            self.assertTrue(isinstance(self.sr_health_check._checks_to_execute[check_name]['thread'], threading.Thread))
            self.assertTrue(isinstance(self.sr_health_check._checks_to_execute[check_name]['check'],
                                       SrHealthReportCheck))

    def test_get_data_from_results_file(self):
        self.assertTrue(isinstance(self.sr_health_check.get_data_from_results_file(), dict))

    def test_save_results_to_result_file_empty_argument(self):
        data_before = self.sr_health_check._current_data
        self.sr_health_check.save_results_to_result_file({})
        self.assertTrue(data_before == self.sr_health_check._current_data)

    def test_save_results_to_result_file_valid_argument(self):
        test_argument = {"test_key": {"test_value": "test"}}
        self.sr_health_check.save_results_to_result_file(test_argument)
        self.assertTrue(set(test_argument) == set(self.sr_health_check.get_data_from_results_file()))

    def test_get_widget(self):
        self.assertTrue(isinstance(self.sr_health_check.get_widget(), QWidget))


if __name__ == "__main__":
    PKG = "sr_health_check"
    rospy.init_node('test_sr_health_check', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_health_check', TestSrHealthCheck)
