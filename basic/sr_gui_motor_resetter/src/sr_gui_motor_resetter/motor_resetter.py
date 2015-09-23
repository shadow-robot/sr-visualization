#!/usr/bin/env python

# Disabling E1002 check since it complains about super for no reason -
# inheriting from QObject
#

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
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import Qt, QThread, SIGNAL, QPoint
from QtGui import QWidget, QMessageBox, QFrame, \
    QHBoxLayout, QCheckBox, QLabel, QColor

from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticArray
from sr_utilities.hand_finder import HandFinder


class MotorFlasher(QThread):

    def __init__(self, parent, nb_motors_to_program, prefix):
        QThread.__init__(self, None)
        self.parent = parent
        self.nb_motors_to_program = nb_motors_to_program
        self.prefix = prefix

    def run(self):
        programmed_motors = 0
        for motor in self.parent.motors:
            if motor.checkbox.checkState() == Qt.Checked:
                try:
                    print(
                        "resetting: realtime_loop/" + self.prefix +
                        "/reset_motor_" + motor.motor_name)
                    self.flasher_service = rospy.ServiceProxy(
                        'realtime_loop/' + self.prefix + '/reset_motor_'
                        + motor.motor_name, Empty)
                    self.flasher_service()
                except rospy.ServiceException, e:
                    self.emit(SIGNAL("failed(QString)"),
                              "Service did not process request: %s" % str(e))
                    return

                programmed_motors += 1
                self.emit(SIGNAL("motor_finished(QPoint)"),
                          QPoint(programmed_motors, 0.0))


class Motor(QFrame):

    def __init__(self, parent, motor_name, motor_index):
        QFrame.__init__(self, parent)

        self.motor_name = motor_name
        self.motor_index = motor_index

        self.layout = QHBoxLayout()

        self.checkbox = QCheckBox(motor_name + " [" + str(motor_index) + "]",
                                  self)
        self.layout.addWidget(self.checkbox)

        self.revision_label = QLabel("")
        self.revision_label.setToolTip("Svn Revision")
        self.layout.addWidget(self.revision_label)

        self.setLayout(self.layout)


class SrGuiMotorResetter(Plugin):

    """
    A gui plugin for resetting motors on the shadow hand.
    """

    def __init__(self, context):
        super(SrGuiMotorResetter, self).__init__(context)
        self.setObjectName('SrGuiMotorResetter')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(
            rospkg.RosPack().get_path('sr_gui_motor_resetter'), 'uis',
            'SrGuiMotorResetter.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrMotorResetterUi')
        context.add_widget(self._widget)

        # setting the prefixes
        self.diag_sub = rospy.Subscriber("diagnostics", DiagnosticArray,
                                         self.diagnostics_callback)

        self._hand_finder = HandFinder()
        hand_parameters = self._hand_finder.get_hand_parameters()
        self._prefix = ""
        for hand in hand_parameters.mapping:
            self._widget.select_prefix.addItem(hand_parameters.mapping[hand])
        if not hand_parameters.mapping:
            rospy.logerr("No hand detected")
            QMessageBox.warning(
                self._widget, "warning", "No hand is detected")
        else:
            self._widget.select_prefix.setCurrentIndex(0)
            self._prefix = hand_parameters.mapping.values()[0]

        self._widget.select_prefix.currentIndexChanged['QString'].connect(
            self.prefix_selected)
        # motors_frame is defined in the ui file with a grid layout
        self.motors = []
        self.motors_frame = self._widget.motors_frame
        self.populate_motors()
        self.progress_bar = self._widget.motors_progress_bar
        self.progress_bar.hide()

        self.server_revision = 0

        # Bind button clicks
        self._widget.btn_select_all.pressed.connect(self.on_select_all_pressed)
        self._widget.btn_select_none.pressed.connect(
            self.on_select_none_pressed)
        self._widget.btn_reset_motors.pressed.connect(
            self.on_reset_motors_pressed)

    def populate_motors(self):

        for i in reversed(range(self.motors_frame.layout().count())):
            self.motors_frame.layout().itemAt(i).widget().setParent(None)
        self.motors = []

        if rospy.has_param(self._prefix + "/joint_to_motor_mapping"):
            joint_to_motor_mapping = rospy.get_param(
                self._prefix + "/joint_to_motor_mapping")
        else:
            QMessageBox.warning(
                self.motors_frame, "Warning", "Couldn't find the " +
                self._prefix + "joint_to_motor_mapping parameter."
                               " Make sure the etherCAT Hand node is running")
            return

        joint_names = [
            ["FFJ0", "FFJ1", "FFJ2", "FFJ3", "FFJ4"],
            ["MFJ0", "MFJ1", "MFJ2", "MFJ3", "MFJ4"],
            ["RFJ0", "RFJ1", "RFJ2", "RFJ3", "RFJ4"],
            ["LFJ0", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5"],
            ["THJ1", "THJ2", "THJ3", "THJ4", "THJ5"],
            ["WRJ1", "WRJ2"]
        ]

        row = 0
        col = 0
        index_jtm_mapping = 0
        for finger in joint_names:
            col = 0
            for joint_name in finger:
                motor_index = joint_to_motor_mapping[index_jtm_mapping]
                if motor_index != -1:
                    motor = Motor(self.motors_frame, joint_name, motor_index)
                    self.motors_frame.layout().addWidget(motor, row, col)
                    self.motors.append(motor)
                    col += 1
                index_jtm_mapping += 1
            row += 1

    def diagnostics_callback(self, msg):
        for status in msg.status:
            for motor in self.motors:
                if motor.motor_name in status.name and self._prefix.replace(
                        "/", "") in status.name:
                    for key_values in status.values:
                        if "Firmware svn revision" in key_values.key:
                            server_current_modified = key_values.value.split(
                                " / ")

                            if server_current_modified[0] \
                                    > self.server_revision:
                                self.server_revision = int(
                                    server_current_modified[0].strip())

                            palette = motor.revision_label.palette()
                            palette.setColor(
                                motor.revision_label.foregroundRole(),
                                Qt.green)
                            if server_current_modified[0].strip() != \
                                    server_current_modified[1].strip():
                                palette.setColor(
                                    motor.revision_label.foregroundRole(),
                                    QColor(255, 170, 23))
                                motor.revision_label.setPalette(palette)

                            if "True" in server_current_modified[2]:
                                palette.setColor(
                                    motor.revision_label.foregroundRole(),
                                    Qt.red)
                                motor.revision_label.setText(
                                    "svn: " + server_current_modified[
                                        1] + " [M]")
                                motor.revision_label.setPalette(palette)
                            else:
                                motor.revision_label.setText(
                                    " svn: " + server_current_modified[1])
                                motor.revision_label.setPalette(palette)

    def on_select_all_pressed(self):
        for motor in self.motors:
            motor.checkbox.setCheckState(Qt.Checked)

    def on_select_none_pressed(self):
        for motor in self.motors:
            motor.checkbox.setCheckState(Qt.Unchecked)

    def on_reset_motors_pressed(self):
        print("Reset motors pressed")
        self.progress_bar.reset()
        nb_motors_to_program = 0
        for motor in self.motors:
            if motor.checkbox.checkState() == Qt.Checked:
                nb_motors_to_program += 1
        if nb_motors_to_program == 0:
            QMessageBox.warning(self._widget, "Warning",
                                "No motors selected for resetting.")
            return
        self.progress_bar.setMaximum(nb_motors_to_program)

        self.motor_flasher = MotorFlasher(self, nb_motors_to_program,
                                          self._prefix)
        self._widget.connect(self.motor_flasher, SIGNAL("finished()"),
                             self.finished_programming_motors)
        self._widget.connect(self.motor_flasher,
                             SIGNAL("motor_finished(QPoint)"),
                             self.one_motor_finished)
        self._widget.connect(self.motor_flasher, SIGNAL("failed(QString)"),
                             self.failed_programming_motors)

        self._widget.setCursor(Qt.WaitCursor)
        self.motors_frame.setEnabled(False)
        self._widget.btn_select_all.setEnabled(False)
        self._widget.btn_select_none.setEnabled(False)
        self.progress_bar.show()
        self._widget.btn_reset_motors.hide()

        self.motor_flasher.start()

    def one_motor_finished(self, point):
        self.progress_bar.setValue(int(point.x()))

    def finished_programming_motors(self):
        self.motors_frame.setEnabled(True)
        self._widget.btn_select_all.setEnabled(True)
        self._widget.btn_select_none.setEnabled(True)
        self._widget.setCursor(Qt.ArrowCursor)
        self.progress_bar.hide()
        self._widget.btn_reset_motors.show()

    def failed_programming_motors(self, message):
        QMessageBox.warning(self.motors_frame, "Warning", message)

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

    def prefix_selected(self, prefix):
        self._prefix = prefix
        self.populate_motors()
