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
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, QPoint
from QtGui import QCursor, QColor
from QtWidgets import QWidget, QShortcut, QMessageBox, QFrame, QHBoxLayout, QCheckBox, QLabel, QFileDialog

from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticArray

from sr_robot_msgs.srv import SimpleMotorFlasher, SimpleMotorFlasherResponse


class MotorBootloader(QThread):

    def __init__(self, parent, nb_motors_to_program):
        QThread.__init__(self, None)
        self.parent = parent
        self.nb_motors_to_program = nb_motors_to_program

    def run(self):
        bootloaded_motors = 0
        firmware_path = self.parent._widget.txt_path.text()
        for motor in self.parent.motors:
            if motor.checkbox.checkState() == Qt.Checked:
                try:
                    self.bootloader_service = rospy.ServiceProxy(
                        'SimpleMotorFlasher', SimpleMotorFlasher)
                    resp = self.bootloader_service(
                        firmware_path.encode('ascii', 'ignore'), motor.motor_index)
                except rospy.ServiceException, e:
                    self.failed['QString'].emit("Service did not process request: %s" % str(e))
                    return

                if resp == SimpleMotorFlasherResponse.FAIL:
                    self.failed['QString'].emit("Bootloading motor " + +"failed")
                bootloaded_motors += 1
                self.motor_finished['QPoint'].emit(QPoint(bootloaded_motors, 0.0))


class Motor(QFrame):

    def __init__(self, parent, motor_name, motor_index):
        QFrame.__init__(self, parent)

        self.motor_name = motor_name
        self.motor_index = motor_index

        self.layout = QHBoxLayout()

        self.checkbox = QCheckBox(
            motor_name + " [" + str(motor_index) + "]", self)
        self.layout.addWidget(self.checkbox)

        self.revision_label = QLabel("")
        self.revision_label.setToolTip("Svn Revision")
        self.layout.addWidget(self.revision_label)

        self.setLayout(self.layout)


class SrGuiBootloader(Plugin):

    """
    A GUI plugin for bootloading the muscle drivers on the etherCAT muscle shadow hand
    """

    def __init__(self, context):
        super(SrGuiBootloader, self).__init__(context)
        self.setObjectName('SrGuiBootloader')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_bootloader'), 'uis', 'SrBootloader.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrMotorResetterUi')
        context.add_widget(self._widget)

        # motors_frame is defined in the ui file with a grid layout
        self.motors = []
        self.motors_frame = self._widget.motors_frame
        self.populate_motors()
        self.progress_bar = self._widget.motors_progress_bar
        self.progress_bar.hide()

        self.server_revision = 0
        self.diag_sub = rospy.Subscriber(
            "diagnostics", DiagnosticArray, self.diagnostics_callback)

        # Bind button clicks
        self._widget.btn_select_bootloader.pressed.connect(
            self.on_select_bootloader_pressed)
        self._widget.btn_select_all.pressed.connect(self.on_select_all_pressed)
        self._widget.btn_select_none.pressed.connect(
            self.on_select_none_pressed)
        self._widget.btn_bootload.pressed.connect(self.on_bootload_pressed)

    def on_select_bootloader_pressed(self):
        """
        Select a hex file to bootload. Hex files must exist in the released_firmaware folder
        """
        path_to_bootloader = "~"
        try:
            path_to_bootloader = os.path.join(rospkg.RosPack().get_path(
                'sr_external_dependencies'), 'compiled_firmware', 'released_firmware')
        except:
            rospy.logwarn(
                "couldn't find the sr_edc_controller_configuration package")

        filter_files = "*.hex"
        filename, _ = QFileDialog.getOpenFileName(
            self._widget.motors_frame, self._widget.tr(
                'Select hex file to bootload'),
            self._widget.tr(
                path_to_bootloader),
            self._widget.tr(filter_files))
        if filename == "":
            return

        self._widget.txt_path.setText(filename)
        self._widget.btn_bootload.setEnabled(True)

    def populate_motors(self):
        """
        Find motors according to joint_to_motor_mapping mapping that must exist on the parameter server
        and add to the list of Motor objects etherCAT hand node must be running
        """
        row = 0
        col = 0
        for motor_index in range(0, 4):
            if motor_index == 0:
                row = 0
                col = 0
            elif motor_index == 1:
                row = 0
                col = 1
            elif motor_index == 2:
                row = 1
                col = 0
            elif motor_index == 3:
                row = 1
                col = 1

            muscle_driver_name = "Muscle driver " + str(motor_index)
            motor = Motor(self.motors_frame, muscle_driver_name, motor_index)
            self.motors_frame.layout().addWidget(motor, row, col)
            self.motors.append(motor)

    def diagnostics_callback(self, msg):
        for status in msg.status:
            for motor in self.motors:
                if motor.motor_name in status.name:
                    for key_values in status.values:
                        if "Firmware svn revision" in key_values.key:
                            server_current_modified = key_values.value.split(" / ")

                            if server_current_modified[0] > self.server_revision:
                                self.server_revision = int(
                                    server_current_modified[0].strip())

                            palette = motor.revision_label.palette()
                            palette.setColor(
                                motor.revision_label.foregroundRole(), Qt.green)
                            if server_current_modified[0].strip() != server_current_modified[1].strip():
                                palette.setColor(
                                    motor.revision_label.foregroundRole(), QColor(255, 170, 23))
                                motor.revision_label.setPalette(palette)

                            if "True" in server_current_modified[2]:
                                palette.setColor(
                                    motor.revision_label.foregroundRole(), Qt.red)
                                motor.revision_label.setText(
                                    "svn: " + server_current_modified[1] + " [M]")
                                motor.revision_label.setPalette(palette)
                            else:
                                motor.revision_label.setText(
                                    " svn: " + server_current_modified[1])
                                motor.revision_label.setPalette(palette)

    def on_select_all_pressed(self):
        """
        Select all motors
        """
        for motor in self.motors:
            motor.checkbox.setCheckState(Qt.Checked)

    def on_select_none_pressed(self):
        """
        Unselect all motors
        """
        for motor in self.motors:
            motor.checkbox.setCheckState(Qt.Unchecked)

    def on_bootload_pressed(self):
        """
        Start programming motors
        """
        self.progress_bar.reset()
        nb_motors_to_program = 0
        for motor in self.motors:
            if motor.checkbox.checkState() == Qt.Checked:
                nb_motors_to_program += 1
        if nb_motors_to_program == 0:
            QMessageBox.warning(
                self._widget, "Warning", "No motors selected for resetting.")
            return
        self.progress_bar.setMaximum(nb_motors_to_program)

        self.motor_bootloader = MotorBootloader(self, nb_motors_to_program)

        self._widget.motor_bootloader.finished.connect(self.finished_programming_motors)
        self._widget.motor_bootloader.motor_finished['QPoint'].connect(self.one_motor_finished)
        self._widget.motor_bootloader.failed['QString'].connect(self.failed_programming_motors)

        self._widget.setCursor(Qt.WaitCursor)
        self.motors_frame.setEnabled(False)
        self._widget.btn_select_all.setEnabled(False)
        self._widget.btn_select_none.setEnabled(False)
        self.progress_bar.show()
        self._widget.btn_bootload.hide()

        self.motor_bootloader.start()

    def one_motor_finished(self, point):
        self.progress_bar.setValue(int(point.x()))

    def finished_programming_motors(self):
        """
        Programming of motors completed
        """
        self.motors_frame.setEnabled(True)
        self._widget.btn_select_all.setEnabled(True)
        self._widget.btn_select_none.setEnabled(True)
        self._widget.setCursor(Qt.ArrowCursor)
        self.progress_bar.hide()
        self._widget.btn_bootload.show()

    def failed_programming_motors(self, message):
        QMessageBox.warning(self._widget.motors_frame, "Warning", message)

    def _unregisterPublisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._unregisterPublisher()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass
