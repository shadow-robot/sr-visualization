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

import os
import rospy
import rospkg
import yaml
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from QtWidgets import *
from QtGui import *
from QtCore import *

# TODO: This will be replaced by imported, actual calibration script
def mock_calibration_script(hand_size_0, hand_size_1, source_position_0, source_position_1, source_position_2):
    return [0, 0, 0]


class SrGuiShadowGloveCalibration(Plugin):
    def __init__(self, context):
        super(SrGuiShadowGloveCalibration, self).__init__(context)
        self.setObjectName('SrGuiShadowGloveCalibration')

        self._widget = QWidget()

        ui_file = os.path.join(
            rospkg.RosPack().get_path('sr_gui_shadow_glove_calibration'), 'uis',
            'SrShadowGloveCalibration.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrShadowGloveCalibrationUI')
        context.add_widget(self._widget)

        self._widget.load_calibration.clicked.connect(self.btn_load_calibration_clicked_)
        self._widget.calibrate.clicked.connect(self.btn_calibrate_clicked_)
        self._widget.save_calibration.clicked.connect(self.btn_save_calibration_clicked_)
        self._widget.set_default.clicked.connect(self.btn_set_default_clicked_)

        self.user_calibration = {}
        self.calibrations_path = '/home/user/shadow_glove_user_calibration_files'
        if not os.path.exists(self.calibrations_path):
            self.calibrations_path= '/home/user'

    def message_box_throw(self, message):
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(message)
            msg.setWindowTitle("Warning!")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()

    def read_user_calibration_from_fields(self):
        self.user_calibration['hand_size']['hl'] = float(self._widget.hand_size_0.text())
        self.user_calibration['hand_size']['hb'] = float(self._widget.hand_size_1.text())
        self.user_calibration['mf_knuckle_to_glove_source_pose']['x'] = float(self._widget.source_position_0.text())
        self.user_calibration['mf_knuckle_to_glove_source_pose']['y'] = float(self._widget.source_position_1.text())
        self.user_calibration['mf_knuckle_to_glove_source_pose']['z'] = float(self._widget.source_position_2.text())
        self.user_calibration['mf_knuckle_to_glove_source_pose']['yaw'] = float(self._widget.source_orientation_0.text())
        self.user_calibration['mf_knuckle_to_glove_source_pose']['pitch'] = float(self._widget.source_orientation_1.text())
        self.user_calibration['mf_knuckle_to_glove_source_pose']['roll'] = float(self._widget.source_orientation_2.text())

    def btn_load_calibration_clicked_(self):
        user_calibration_file_path = QFileDialog.getOpenFileName(self._widget, 'Open file', self.calibrations_path, 'YAML file (*.yaml)')[0]
        try:
            with open("{}".format(user_calibration_file_path)) as f:
                self.user_calibration = yaml.load(f)
        except IOError, yaml.reader.ReaderError:
            self.message_box_throw("Wrong file type or format!")
        else:
            self._widget.hand_size_0.clear()
            self._widget.hand_size_1.clear()
            self._widget.source_position_0.clear()
            self._widget.source_position_1.clear()
            self._widget.source_position_2.clear()
            self._widget.source_orientation_0.clear()
            self._widget.source_orientation_1.clear()
            self._widget.source_orientation_2.clear()
            self._widget.hand_size_0.insert(str(self.user_calibration['hand_size']['hl']))
            self._widget.hand_size_1.insert(str(self.user_calibration['hand_size']['hb']))
            self._widget.source_position_0.insert(str(self.user_calibration['mf_knuckle_to_glove_source_pose']['x']))
            self._widget.source_position_1.insert(str(self.user_calibration['mf_knuckle_to_glove_source_pose']['y']))
            self._widget.source_position_2.insert(str(self.user_calibration['mf_knuckle_to_glove_source_pose']['z']))
            self._widget.source_orientation_0.insert(str(self.user_calibration['mf_knuckle_to_glove_source_pose']['yaw']))
            self._widget.source_orientation_1.insert(str(self.user_calibration['mf_knuckle_to_glove_source_pose']['pitch']))
            self._widget.source_orientation_2.insert(str(self.user_calibration['mf_knuckle_to_glove_source_pose']['roll']))

    def btn_calibrate_clicked_(self):
        try:
            hand_size_0 = float(self._widget.hand_size_0.text())
            hand_size_1 = float(self._widget.hand_size_1.text())
            source_position_0 = float(self._widget.source_position_0.text())
            source_position_1 = float(self._widget.source_position_1.text())
            source_position_2 = float(self._widget.source_position_2.text())
        except ValueError:
            self.message_box_throw("Please correctly fill the necessary fields.")
        else:
            # TODO: Replace by actual calibration script call when it's available
            source_orientation = mock_calibration_script(hand_size_0, hand_size_1, source_position_0, source_position_1, source_position_2)
            source_orientation = [str(value_float) for value_float in source_orientation]
            self._widget.source_orientation_0.clear()
            self._widget.source_orientation_1.clear()
            self._widget.source_orientation_2.clear()
            self._widget.source_orientation_0.insert(source_orientation[0])
            self._widget.source_orientation_1.insert(source_orientation[1])
            self._widget.source_orientation_2.insert(source_orientation[2])

    def btn_save_calibration_clicked_(self):
        output_file_path = QFileDialog.getSaveFileName(self._widget, 'Save File',
                                                       self.calibrations_path,
                                                       'YAML file (*.yaml)')[0]
        if not output_file_path:
            return
        if not output_file_path.endswith(".yaml"):
            output_file_path += ".yaml"
        self.read_user_calibration_from_fields()
        with open(r'{}'.format(output_file_path), 'w+') as f:
            yaml.dump(self.user_calibration, f, default_flow_style=False)

    def btn_set_default_clicked_(self):
        if not '/home/user/shadow_glove_user_calibration_files' == self.calibrations_path:
            self.message_box_throw("Since the aurora-created directory for calibrations does not exist,"
                                    " most likely this action will not have any effect. Make sure you"
                                    " installed all the software for the hand correctly.")
        chosen_calibration_path = QFileDialog.getOpenFileName(self._widget, 'Open file', self.calibrations_path, 'YAML file (*.yaml)')[0]
        create_symlink_command = 'ln -sf {} {}/default_calibration'.format(chosen_calibration_path, self.calibrations_path)
        os.system(create_symlink_command)