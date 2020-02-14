# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QIcon
from QtWidgets import QShortcut, QMessageBox, QWidget
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest, LoadController
from sr_robot_msgs.srv import ChangeControlType
from sr_robot_msgs.msg import ControlType


class SrGuiChangeControllers(Plugin):

    """
    A rosgui plugin for loading the different controllers
    """
    ICON_DIR = os.path.join(
        rospkg.RosPack().get_path('sr_visualization_icons'), 'icons')
    CONTROLLER_ON_ICON = QIcon(os.path.join(ICON_DIR, 'green.png'))
    CONTROLLER_OFF_ICON = QIcon(os.path.join(ICON_DIR, 'red.png'))

    controllers = {
        "valve": ["sh_ffj0_muscle_valve_controller",
                  "sh_ffj3_muscle_valve_controller",
                  "sh_ffj4_muscle_valve_controller",
                  "sh_mfj0_muscle_valve_controller",
                  "sh_mfj3_muscle_valve_controller",
                  "sh_mfj4_muscle_valve_controller",
                  "sh_rfj0_muscle_valve_controller",
                  "sh_rfj3_muscle_valve_controller",
                  "sh_rfj4_muscle_valve_controller",
                  "sh_lfj0_muscle_valve_controller",
                  "sh_lfj3_muscle_valve_controller",
                  "sh_lfj4_muscle_valve_controller",
                  "sh_lfj5_muscle_valve_controller",
                  "sh_thj1_muscle_valve_controller",
                  "sh_thj2_muscle_valve_controller",
                  "sh_thj3_muscle_valve_controller",
                  "sh_thj4_muscle_valve_controller",
                  "sh_thj5_muscle_valve_controller",
                  "sh_wrj1_muscle_valve_controller",
                  "sh_wrj2_muscle_valve_controller"],
        "position": ["sh_ffj0_muscle_position_controller",
                     "sh_ffj3_muscle_position_controller",
                     "sh_ffj4_muscle_position_controller",
                     "sh_mfj0_muscle_position_controller",
                     "sh_mfj3_muscle_position_controller",
                     "sh_mfj4_muscle_position_controller",
                     "sh_rfj0_muscle_position_controller",
                     "sh_rfj3_muscle_position_controller",
                     "sh_rfj4_muscle_position_controller",
                     "sh_lfj0_muscle_position_controller",
                     "sh_lfj3_muscle_position_controller",
                     "sh_lfj4_muscle_position_controller",
                     "sh_lfj5_muscle_position_controller",
                     "sh_thj1_muscle_position_controller",
                     "sh_thj2_muscle_position_controller",
                     "sh_thj3_muscle_position_controller",
                     "sh_thj4_muscle_position_controller",
                     "sh_thj5_muscle_position_controller",
                     "sh_wrj1_muscle_position_controller",
                     "sh_wrj2_muscle_position_controller"],
        "stop": []}

    def __init__(self, context):
        super(SrGuiChangeControllers, self).__init__(context)
        self.setObjectName('SrGuiChangeControllers')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_change_muscle_controllers'), 'uis', 'SrChangeControllers.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrChangeControllersUi')
        context.add_widget(self._widget)

        self.list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        self.switch_controllers = rospy.ServiceProxy(
                'controller_manager/switch_controller', SwitchController)

        # Setting the initial state of the controller buttons
        self._widget.btn_valve.setIcon(self.CONTROLLER_OFF_ICON)
        self._widget.btn_valve.setChecked(False)
        self._widget.btn_position.setIcon(self.CONTROLLER_OFF_ICON)
        self._widget.btn_position.setChecked(False)

        # attaching the button press event to their actions
        self._widget.btn_stop.pressed.connect(self.on_stop_ctrl_clicked_)
        self._widget.btn_valve.pressed.connect(self.on_valve_ctrl_clicked_)
        self._widget.btn_position.pressed.connect(
            self.on_position_ctrl_clicked_)

    def on_stop_ctrl_clicked_(self):
        """
        Stop the controller
        """
        self._widget.btn_stop.setEnabled(False)
        self._widget.btn_valve.setIcon(self.CONTROLLER_OFF_ICON)
        self._widget.btn_valve.setChecked(False)
        self._widget.btn_position.setIcon(self.CONTROLLER_OFF_ICON)
        self._widget.btn_position.setChecked(False)
        self.change_ctrl("stop")
        self._widget.btn_stop.setEnabled(True)

    def on_valve_ctrl_clicked_(self):
        """
        Switch to valve control
        """
        self._widget.btn_valve.setEnabled(False)
        if not self._widget.btn_valve.isChecked():
            self._widget.btn_valve.setIcon(self.CONTROLLER_ON_ICON)
            self._widget.btn_valve.setChecked(True)
            self._widget.btn_position.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.btn_position.setChecked(False)
            rospy.loginfo("Valve checked: " + str(
                self._widget.btn_valve.isChecked()))
            self.change_ctrl("valve")
        else:
            self._widget.btn_valve.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.btn_valve.setChecked(False)
            rospy.loginfo("Valve checked: " + str(
                self._widget.btn_valve.isChecked()))
            self.change_ctrl("stop")
        self._widget.btn_valve.setEnabled(True)

    def on_position_ctrl_clicked_(self):
        """
        Switch to position control
        """
        self._widget.btn_position.setEnabled(False)
        if not self._widget.btn_position.isChecked():
            self._widget.btn_position.setIcon(self.CONTROLLER_ON_ICON)
            self._widget.btn_position.setChecked(True)
            self._widget.btn_valve.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.btn_valve.setChecked(False)
            rospy.loginfo("Position checked: " + str(
                self._widget.btn_position.isChecked()))
            self.change_ctrl("position")
        else:
            self._widget.btn_position.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.btn_position.setChecked(False)
            rospy.loginfo("Position checked: " + str(
                self._widget.btn_position.isChecked()))
            self.change_ctrl("stop")
        self._widget.btn_position.setEnabled(True)

    def change_ctrl(self, controller):
        """
        Switch controller type
        """
        success = True

        try:
            resp1 = self.list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            current_controllers = [
                c.name for c in resp1.controller if c.state == "running"]
            all_loaded_controllers = [c.name for c in resp1.controller]

            controllers_to_start = self.controllers[controller]
            controllers_to_start.append('joint_state_controller')

            load_controllers = rospy.ServiceProxy(
                'controller_manager/load_controller', LoadController)
            for load_control in controllers_to_start:
                if load_control not in all_loaded_controllers:
                    try:
                        resp1 = load_controllers(load_control)
                    except rospy.ServiceException:
                        success = False
                    if not resp1.ok:
                        success = False

            req.start_controllers = controllers_to_start
            req.stop_controllers = current_controllers
            req.strictness = SwitchControllerRequest.BEST_EFFORT
            req.start_asap = False
            req.timeout = 0.0

            try:
                resp1 = self.switch_controller.call(req)
            except rospy.ServiceException:
                success = False

            if not resp1.ok:
                success = False

        if not success:
            rospy.logwarn(
                "Failed to change some of the controllers. This is normal if this is not a 5 finger hand.")

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
