import os
import rospy
import rospkg
import yaml
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from QtWidgets import *
from QtGui import *
from QtCore import *


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
        user_calibration_file_path = QFileDialog.getOpenFileName(self._widget, 'Open file', self.calibrations_path)[0]
        try:
            with open("{}".format(user_calibration_file_path)) as f:
                self.user_calibration = yaml.load(f)
                rospy.logwarn(self.user_calibration)
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