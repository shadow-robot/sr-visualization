#!/usr/bin/env python
#
# Copyright 2012 Shadow Robot Company Ltd.
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
#

import os, sys, rospkg, rospy

from xml.etree import ElementTree as ET

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QWidget, QShortcut, QMessageBox, QFrame
from pr2_mechanism_msgs.srv import ListControllers, SwitchController, LoadController

from sr_gui_joint_slider.jointSlider import JointController, Joint, CANHandSlider, EtherCATHandSlider, ArmSlider, CANHandSelectionSlider, EtherCATSelectionSlider, ArmSelectionSlider
from sr_robot_lib.etherCAT_hand_lib import EtherCAT_Hand_Lib
from sr_hand.shadowhand_ros import ShadowHand_ROS

class SrGuiJointSlider(Plugin):

    def __init__(self, context):
        super(SrGuiJointSlider, self).__init__(context)
        self.setObjectName('SrGuiJointSlider')

        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_joint_slider'), 'uis', 'SrJointSlider.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('SrJointSliderUi')
        context.add_widget(self._widget)

        #read the xml configuration file
        config_file = os.path.join(rospkg.RosPack().get_path('sr_gui_joint_slider'), 'model', 'slide_joints.xml')
        self.tree = ET.ElementTree()
        self.tree.parse(config_file)
        self.robots = None

        self.sliders = list()
        self.selection_slider = None

        robot_types = self.get_robot_types(self.tree)
        self._widget.comboBox.addItems(robot_types)
        self._widget.comboBox.setCurrentIndex(-1)

        self.is_active = True
        self.robot_lib_CAN = None
        self.robot_lib_eth = None

        self._widget.comboBox.activated.connect(self.on_robot_type_changed_)
        self._widget.reloadButton.pressed.connect(self.on_reload_button_cicked_)
        self._widget.refreshButton.pressed.connect(self.on_refresh_button_cicked_)
        self._widget.sliderReleaseCheckBox.stateChanged.connect(self.on_slider_release_checkbox_clicked_)

    def _unregister(self):
        if self.robot_lib_eth is not None:
            self.robot_lib_eth.on_close()

    def shutdown_plugin(self):
        self._unregister()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass

    def get_robot_types(self, config):
        if sys.version_info < (2, 7):
            self.robots = list(config.getiterator("robot"))
        else:
            self.robots = list(config.iter("robot"))
        robot_types = list()
        for element in self.robots:
            robot_types.append(element.attrib["name"])
        return robot_types

    def on_robot_type_changed_(self):
        #We first read the config from the file into a joints list
        j = self.robots[self._widget.comboBox.currentIndex()].find("joints")
        if sys.version_info < (2, 7):
            config_joints = list(j.getiterator("joint"))
        else:
            #For Python version 2.7 and later
            config_joints = list(j.iter("joint"))

        #Read the joints configuration
        self.joints = list()
        for config_joint in config_joints:
            if sys.version_info < (2, 7):
                config_joint_controllers = list(config_joint.getiterator("controller"))
            else:
                config_joint_controllers = list(config_joint.iter("controller"))
            joint_controllers = list()
            for config_controller in config_joint_controllers:
                name = config_controller.attrib["name"]
                command_topic = config_controller.findtext("command_topic")
                msg_type = config_controller.findtext("msg_type")
                min = int(config_controller.findtext("min"))
                max = int(config_controller.findtext("max"))
                controller = JointController(name, command_topic, msg_type, min, max)
                joint_controllers.append(controller)
            name = config_joint.attrib["name"]
            joint = Joint(name, joint_controllers)
            self.joints.append(joint)

        #Clear existing slider widgets from layout
        self.delete_old_sliders_()

        #Load the correct robot library
        self.load_robot_library_()
        
        self._widget.sliderReleaseCheckBox.setCheckState(Qt.Unchecked)

        if self.is_active:
            #Create and load the new slider widgets
            self.load_new_sliders_()
            
        self._widget.reloadButton.setEnabled(True)

    def on_reload_button_cicked_(self):
        #Clear existing slider widgets from layout
        self.delete_old_sliders_()

        #Load the correct robot library
        self.load_robot_library_()
        
        self._widget.sliderReleaseCheckBox.setCheckState(Qt.Unchecked)

        if self.is_active:
            #Create and load the new slider widgets
            self.load_new_sliders_()
    
    def on_refresh_button_cicked_(self):
        #Call refresh for every slider
        for slider in self.sliders:
            slider.refresh()
    
    def on_slider_release_checkbox_clicked_(self, state):
        if state == Qt.Checked:
            #Call set_new_slider_behaviour for every slider
            # the tracking behaviour will be set to false
            for slider in self.sliders:
                slider.set_new_slider_behaviour(False)
        else:
            #Call set_new_slider_behaviour for every slider
            for slider in self.sliders:
                slider.set_new_slider_behaviour(True)
            
        

    def delete_old_sliders_(self):
        #Clear existing slider widgets from layout
        for old_slider in self.sliders:
            self._widget.horizontalLayout.removeWidget(old_slider)
            old_slider.close()
            old_slider.deleteLater()

        #Empty the slider list
        self.sliders = list()

        if(self.selection_slider is not None):
            self._widget.horizontalLayout.removeWidget(self.selection_slider)
            self.selection_slider.close()
            self.selection_slider.deleteLater()
            self.selection_slider = None

    def load_robot_library_(self):
        #Load the correct robot library
        if self._widget.comboBox.currentText() in ["CAN Hand", "Arm"]:
            self.is_active = True
            if self.robot_lib_CAN is None:
                self.robot_lib_CAN = ShadowHand_ROS()
        elif self._widget.comboBox.currentText() == "EtherCAT Hand":
            self.is_active = True
            if self.robot_lib_eth is not None:
                self.robot_lib_eth.on_close()
                self.robot_lib_eth = None
            if self.robot_lib_eth is None:
                self.robot_lib_eth = EtherCAT_Hand_Lib()
                if not self.robot_lib_eth.activate_joint_states():
                    btn_pressed = QMessageBox.warning(self._widget, "Warning", "The EtherCAT Hand node doesn't seem to be running. Try to reload the sliders when it is.")
                    self.is_active = False
                    if self.robot_lib_eth is not None:
                        self.robot_lib_eth.on_close()
                        self.robot_lib_eth = None
        else:
            rospy.logwarn("Unknown robot name: " + self._widget.comboBox.currentText())

    def load_new_sliders_(self):
        #Create the new slider widgets
        self.sliders = list()
        for joint in self.joints:
            slider = None
            slider_ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_joint_slider'), 'uis', 'Slider.ui')
            if self._widget.comboBox.currentText() == "CAN Hand":
                slider = CANHandSlider(joint, slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
            elif self._widget.comboBox.currentText() == "EtherCAT Hand":
                try:
                    slider = EtherCATHandSlider(joint, slider_ui_file, self.robot_lib_eth, self, self._widget.scrollAreaWidgetContents)
                except Exception, e:
                    rospy.loginfo(e)
            elif self._widget.comboBox.currentText() == "Arm":
                slider = ArmSlider(joint, slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
            else:
                rospy.logwarn("Unknown robot name: " + self._widget.comboBox.currentText())

            if slider != None:
                slider.setMaximumWidth(100)
                #Load the new slider
                self._widget.horizontalLayout.addWidget(slider)
                #Put the slider in the list
                self.sliders.append(slider)

        #Create the slider to move all the selected joint sliders
        selection_slider_ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_joint_slider'), 'uis', 'SelectionSlider.ui')
        if self._widget.comboBox.currentText() == "CAN Hand":
            self.selection_slider = CANHandSelectionSlider("Change sel.", 0, 100, selection_slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
        elif self._widget.comboBox.currentText() == "EtherCAT Hand":
            self.selection_slider = EtherCATSelectionSlider("Change sel.", 0, 100, selection_slider_ui_file, self, self._widget.scrollAreaWidgetContents)
        elif self._widget.comboBox.currentText() == "Arm":
            self.selection_slider = ArmSelectionSlider("Change sel.", 0, 100, selection_slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
        else:
            rospy.logwarn("Unknown robot name: " + self._widget.comboBox.currentText())

        self.selection_slider.setMaximumWidth(100)
        self._widget.horizontalLayout.addWidget(self.selection_slider)
