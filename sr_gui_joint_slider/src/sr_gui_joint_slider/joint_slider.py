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

import os
import rospkg
import rospy

from xml.etree import ElementTree as ET

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import Qt
from QtGui import QWidget, QMessageBox

from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointControllerState
from sr_robot_msgs.msg import JointControllerState as SrJointControllerState
from sr_robot_msgs.msg import JointMusclePositionControllerState

from sr_gui_joint_slider.sliders import JointController, Joint, EtherCATHandSlider, EtherCATSelectionSlider
from sr_robot_lib.etherCAT_hand_lib import EtherCAT_Hand_Lib
from sr_hand.shadowhand_ros import ShadowHand_ROS

class SrGuiJointSlider(Plugin):
    """
    A rosgui plugin to change the position of the different joints
    """
    
    
    controller_state_types = {"sr_mechanism_controllers/SrhJointPositionController": ("position", JointControllerState),
                             "sr_mechanism_controllers/SrhEffortJointController": ("effort", JointControllerState),
                             "sr_mechanism_controllers/SrhJointVelocityController": ("velocity", JointControllerState),
                             "sr_mechanism_controllers/SrhMixedPositionVelocityJointController": ("position", SrJointControllerState),
                             "sr_mechanism_controllers/SrhMuscleJointPositionController": ("position", JointMusclePositionControllerState)} 
    
    def __init__(self, context):
        super(SrGuiJointSlider, self).__init__(context)
        self.setObjectName('SrGuiJointSlider')
        
        self._robot_description_xml_root = None

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
        self.joints = []

        self.sliders = []
        self.selection_slider = None

        robot_types = ["EtherCAT Hand"]
        self._widget.comboBox.addItems(robot_types)
        self._widget.comboBox.setCurrentIndex(-1)

        self.is_active = True
        self.robot_lib_CAN = None
        self.robot_lib_eth = None

        #self._widget.comboBox.activated.connect(self.on_robot_type_changed_)
        self._widget.reloadButton.pressed.connect(self.on_reload_button_cicked_)
        self._widget.refreshButton.pressed.connect(self.on_refresh_button_cicked_)
        self._widget.sliderReleaseCheckBox.stateChanged.connect(self.on_slider_release_checkbox_clicked_)

	# Default to ethercat hand sliders
        if "EtherCAT Hand" in robot_types:
            self._widget.comboBox.setCurrentIndex(robot_types.index("EtherCAT Hand"))
            self.on_robot_type_changed_()

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
        self.robots = list(config.iter("robot"))
        robot_types = list()
        for element in self.robots:
            robot_types.append(element.attrib["name"])
        return robot_types

    def on_robot_type_changed_(self):
        self._widget.reloadButton.setEnabled(True)
        self.on_reload_button_cicked_()
#         """
#         Read joints configuration the config from the file
#         Clear existing slider widgets from layout
#         Load the correct robot library
#         Create and load the new slider widgets
#         """
#  
#         #We first read the config from the file into a joints list
#         j = self.robots[self._widget.comboBox.currentIndex()].find("joints")
#         config_joints = list(j.iter("joint"))
#  
#         #Read the joints configuration
#         self.joints = []
#         for config_joint in config_joints:
#             config_joint_controllers = list(config_joint.iter("controller"))
#             joint_controllers = list()
#             for config_controller in config_joint_controllers:
#                 name = config_controller.attrib["name"]
#                 command_topic = config_controller.findtext("command_topic")
#                 msg_type = config_controller.findtext("msg_type")
#                 min = int(config_controller.findtext("min"))
#                 max = int(config_controller.findtext("max"))
#                 controller = JointController(name, command_topic, msg_type, min, max)
#                 joint_controllers.append(controller)
#             name = config_joint.attrib["name"]
#             joint = Joint(name, joint_controllers)
#             self.joints.append(joint)
# 
#         #Clear existing slider widgets from layout
#         self.delete_old_sliders_()
# 
#         #Load the correct robot library
#         self.load_robot_library_()
# 
#         self._widget.sliderReleaseCheckBox.setCheckState(Qt.Unchecked)
# 
#         if self.is_active:
#             #Create and load the new slider widgets
#             self.load_new_sliders_()
# 
#         self._widget.reloadButton.setEnabled(True)

    def on_reload_button_cicked_(self):
        """
        Clear existing slider widgets from layout
        Load the correct robot library
        Create and load the new slider widgets
        """

        self._load_robot_description()
        controllers = self.get_current_controllers()
        
        self.joints = self._create_joints(controllers)
        
        

        self.delete_old_sliders_()

        self.load_robot_library_()

        self._widget.sliderReleaseCheckBox.setCheckState(Qt.Unchecked)

        if self.is_active:
            self.load_new_sliders_()

    def on_refresh_button_cicked_(self):
        """
        Call refresh for every slider
        """
        for slider in self.sliders:
            slider.refresh()

    def on_slider_release_checkbox_clicked_(self, state):
        """
        Set tracking behaviour of each slider to false if checkbox is checked, true otherwise
        """

        if state == Qt.Checked:
            for slider in self.sliders:
                slider.set_new_slider_behaviour(False)
        else:
            for slider in self.sliders:
                slider.set_new_slider_behaviour(True)



    def delete_old_sliders_(self):
        """
        Clear existing slider widgets from layout
        Empty the slider list
        """
        for old_slider in self.sliders:
            self._widget.horizontalLayout.removeWidget(old_slider)
            old_slider.close()
            old_slider.deleteLater()

        self.sliders = []

        if(self.selection_slider is not None):
            self._widget.horizontalLayout.removeWidget(self.selection_slider)
            self.selection_slider.close()
            self.selection_slider.deleteLater()
            self.selection_slider = None

    def load_robot_library_(self):
        """
        Load the correct robot library
        """
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
                    QMessageBox.warning(self._widget, "Warning", "The EtherCAT Hand node doesn't seem to be running. Try reloading the sliders when it is.")
                    self.is_active = False
                    if self.robot_lib_eth is not None:
                        self.robot_lib_eth.on_close()
                        self.robot_lib_eth = None
        else:
            rospy.logwarn("Unknown robot name: " + self._widget.comboBox.currentText())

    def load_new_sliders_(self):
        """
        Create the new slider widgets
        Load the new slider
        Put the slider in the list
        """
        self.sliders = list()
        for joint in self.joints:
            slider = None
            slider_ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_joint_slider'), 'uis', 'Slider.ui')
#             if self._widget.comboBox.currentText() == "CAN Hand":
#                 slider = CANHandSlider(joint, slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
#             elif self._widget.comboBox.currentText() == "EtherCAT Hand":
            try:
                slider = EtherCATHandSlider(joint, slider_ui_file, self, self._widget.scrollAreaWidgetContents)
            except Exception, e:
                rospy.loginfo(e)
#             elif self._widget.comboBox.currentText() == "Arm":
#                 slider = ArmSlider(joint, slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
#             else:
#                 rospy.logwarn("Unknown robot name: " + self._widget.comboBox.currentText())

            if slider != None:
                slider.setMaximumWidth(100)
                #Load the new slider
                self._widget.horizontalLayout.addWidget(slider)
                #Put the slider in the list
                self.sliders.append(slider)

        #Create the slider to move all the selected joint sliders
        selection_slider_ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_joint_slider'), 'uis', 'SelectionSlider.ui')
#         if self._widget.comboBox.currentText() == "CAN Hand":
#             self.selection_slider = CANHandSelectionSlider("Change sel.", 0, 100, selection_slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
#         elif self._widget.comboBox.currentText() == "EtherCAT Hand":
        self.selection_slider = EtherCATSelectionSlider("Change sel.", 0, 100, selection_slider_ui_file, self, self._widget.scrollAreaWidgetContents)
#         elif self._widget.comboBox.currentText() == "Arm":
#             self.selection_slider = ArmSelectionSlider("Change sel.", 0, 100, selection_slider_ui_file, self.robot_lib_CAN, self, self._widget.scrollAreaWidgetContents)
#         else:
#             rospy.logwarn("Unknown robot name: " + self._widget.comboBox.currentText())

        self.selection_slider.setMaximumWidth(100)
        self._widget.horizontalLayout.addWidget(self.selection_slider)

    def get_current_controllers(self):
        """
        @return: list of current controllers
        """
        success = True
        list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            return [c.name for c in resp1.controller if c.state == "running"]
        else:
            rospy.loginfo("Couldn't get list of controllers from controller_manager/list_controllers service")
            return []
        
    def _load_robot_description(self):
        """
        Load the description from the param named in the edit as an ET element.
        Sets self._robot_description_xml_root to the element.
        """
        name = "robot_description"    #  self.robot_description_edit.text()
        self._robot_description_xml_root = None
        try:
            xml = rospy.get_param(name)
            self._robot_description_xml_root = ET.fromstring(xml)
        except KeyError as e:
            rospy.logerr("Failed to get robot description from param %s : %s"%(name,e))
            return
        except:
            raise

    def _get_joint_min_max_vel(self, jname):
        """Get the min and max from the robot description for a given joint."""
        root = self._robot_description_xml_root
        if root is not None:
            limit = root.findall(".//joint[@name='" + jname + "']/limit")
            if limit is None or len(limit) == 0:
                # Handles upper case joint names in the model. e.g. the E1 shadowhand
                limit = root.findall(".//joint[@name='" + jname.upper() + "']/limit")
            if limit is not None and len(limit) > 0:
                return (float(limit[0].attrib['lower']), float(limit[0].attrib['upper']), float(limit[0].attrib['velocity']))
            else:
                rospy.logerr("Limit not found for joint %s", jname)
        else:
            rospy.logerr("robot_description_xml_root == None")
        return (None, None, None)
    
    def _get_joint_min_max_vel_special(self, jname):
        if "J0" in jname:
            jname1 = jname.replace("J0", "J1")
            jname2 = jname.replace("J0", "J2")
            min1, max1, vel1 = self._get_joint_min_max_vel(jname1)
            min2, max2, vel2 = self._get_joint_min_max_vel(jname2)
            return (min1 + min2, max1 + max2, vel1 + vel2)
        else:
            return self._get_joint_min_max_vel(jname)
    
    def _create_joints(self, controllers):
        joints = []
        for controller in controllers:
            if rospy.has_param(controller):
                ctrl_params = rospy.get_param(controller)
                controller_type = ctrl_params["type"]
                if controller_type in self.controller_state_types:
                    controller_state_type = self.controller_state_types[controller_type][1]
                    controller_category = self.controller_state_types[controller_type][0]
                    joint_controller = JointController(controller, controller_type, controller_state_type, controller_category)
                    rospy.loginfo("controller category: %s", controller_category)
                    joint_name = ctrl_params["joint"]
                    min, max, vel = self._get_joint_min_max_vel_special(joint_name)
                    joint = Joint(joint_name, min, max, vel, joint_controller)
                else:
                    rospy.logwarn("Controller %s of type %s not supported", controller, controller_type)
                    continue
            else:
                rospy.logwarn("Parameters for controller %s not found", controller)
                continue
            
            joints.append(joint)
        return joints
