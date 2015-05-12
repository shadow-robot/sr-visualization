#!/usr/bin/env python
import os
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from QtGui import QMessageBox, QWidget, QIcon
import rospy
import rospkg
from sr_robot_msgs.srv import RobotTeachMode, RobotTeachModeRequest, RobotTeachModeResponse


class SrGuiTeachMode(Plugin):
    """
    A rosgui plugin for loading the different controllers
    """

    def __init__(self, context):
        super(SrGuiTeachMode, self).__init__(context)
        self.setObjectName('SrGuiTeachMode')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_teach_mode'), 'uis', 'SrTeachMode.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrTeachModeUi')
        context.add_widget(self._widget)

        self._rh_teach_buttons = []
        self._lh_teach_buttons = []
        self._ra_teach_buttons = []
        self._la_teach_buttons = []

        # rh group
        self._rh_teach_buttons.append(self._widget.radioButton_1)
        self._rh_teach_buttons.append(self._widget.radioButton_2)
        self._rh_teach_buttons.append(self._widget.radioButton_3)
        self._widget.radioButton_1.toggled.connect(self.teach_mode_button_toggled_rh)
        self._widget.radioButton_2.toggled.connect(self.teach_mode_button_toggled_rh)
        self._widget.radioButton_3.toggled.connect(self.teach_mode_button_toggled_rh)

        # lh group
        self._lh_teach_buttons.append(self._widget.radioButton_4)
        self._lh_teach_buttons.append(self._widget.radioButton_5)
        self._lh_teach_buttons.append(self._widget.radioButton_6)
        self._widget.radioButton_4.toggled.connect(self.teach_mode_button_toggled_lh)
        self._widget.radioButton_5.toggled.connect(self.teach_mode_button_toggled_lh)
        self._widget.radioButton_6.toggled.connect(self.teach_mode_button_toggled_lh)

        # ra group
        self._ra_teach_buttons.append(self._widget.radioButton_7)
        self._ra_teach_buttons.append(self._widget.radioButton_8)
        self._ra_teach_buttons.append(self._widget.radioButton_9)
        self._widget.radioButton_7.toggled.connect(self.teach_mode_button_toggled_ra)
        self._widget.radioButton_8.toggled.connect(self.teach_mode_button_toggled_ra)
        self._widget.radioButton_9.toggled.connect(self.teach_mode_button_toggled_ra)

        # la group
        self._la_teach_buttons.append(self._widget.radioButton_10)
        self._la_teach_buttons.append(self._widget.radioButton_11)
        self._la_teach_buttons.append(self._widget.radioButton_12)
        self._widget.radioButton_10.toggled.connect(self.teach_mode_button_toggled_la)
        self._widget.radioButton_11.toggled.connect(self.teach_mode_button_toggled_la)
        self._widget.radioButton_12.toggled.connect(self.teach_mode_button_toggled_la)

    def teach_mode_button_toggled_rh(self, checked):
        self.teach_mode_button_toggled(checked, "right_hand", self._rh_teach_buttons)

    def teach_mode_button_toggled_lh(self, checked):
        self.teach_mode_button_toggled(checked, "left_hand", self._lh_teach_buttons)

    def teach_mode_button_toggled_ra(self, checked):
        self.teach_mode_button_toggled(checked, "right_arm", self._ra_teach_buttons)

    def teach_mode_button_toggled_la(self, checked):
        self.teach_mode_button_toggled(checked, "left_arm", self._la_teach_buttons)

    def teach_mode_button_toggled(self, checked, robot, buttons):
        if checked:
            if buttons[0].isChecked():
                mode = RobotTeachModeRequest.TRAJECTORY_MODE
            elif buttons[1].isChecked():
                mode = RobotTeachModeRequest.TEACH_MODE
            elif buttons[2].isChecked():
                mode = RobotTeachModeRequest.POSITION_MODE
            else:
                rospy.logerr("None of the buttons checked for robot %s", robot)
                return

            rospy.loginfo("Changing robot %s to mode %d", robot, mode)
            self.change_teach_mode(mode, robot)

    @staticmethod
    def change_teach_mode(mode, robot):

        teach_mode_client = rospy.ServiceProxy('/teach_mode', RobotTeachMode)

        req = RobotTeachModeRequest()
        req.teach_mode = mode
        req.robot = robot
        try:
            resp = teach_mode_client(req)
            if resp.result == RobotTeachModeResponse.ERROR:
                rospy.logerr("Failed to change robot %s to mode %d", robot, mode)
            else:
                rospy.loginfo("Changed robot %s to mode %d Result = %d", robot, mode, resp.result)
        except rospy.ServiceException:
            rospy.logerr("Failed to call service teach_mode")
