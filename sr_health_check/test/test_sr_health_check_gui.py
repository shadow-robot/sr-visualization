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


from sr_health_check.sr_health_check_gui import SrGuiHealthCheck


class TestSrHealthCheckGui(TestCase, SrHealthcheckGui):

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_sr_gui")
        super(SrHealthcheckGui, self).__init__()

    def test_initialize_checks(self):
        self.initialize_checks()
        for check_name in self._check_names:
            self.assertTrue(isinstance(self._checks_to_execute[name]['thread'], threading.Thread))
            self.assertTrue(isinstance(self._checks_to_execute[name]['check'], SrHealthReportCheck))

    def test_get_data_from_results_file(self):
        self.assertTrue(isinstance(self.get_data_from_results_file(), dict))

    def test_save_results_to_result_file_empty_argument(self):
        data_before = self._current_data
        self.save_results_to_result_file({})
        self.assertTrue(data_before = self._current_data)

    def test_save_results_to_result_file_valid_argument(self):
        data_before = self._current_data
        test_argument = {"test_key": "test_value"}
        self.save_results_to_result_file(test_argument)
        self.assertTrue(data_before == set(data_before)^set(self.get_data_from_results_file()))

    def test_get_widget(self):
        self.assertTrue(isinstance(self.get_widget() == QWidget))

