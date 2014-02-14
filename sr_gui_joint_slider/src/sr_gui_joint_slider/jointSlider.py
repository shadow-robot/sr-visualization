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

import roslib
roslib.load_manifest('sr_gui_joint_slider')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

#import xml.dom.minidom
from xml.etree import ElementTree as ET

from PyQt4 import QtCore, QtGui, Qt
from QtGui import QShortcut, QMessageBox, QFrame
from pr2_mechanism_msgs.srv import ListControllers, SwitchController, LoadController
from sr_robot_msgs.msg import sendupdate, joint
from std_msgs.msg import Float64
from math import radians, degrees

class JointController():
    """
    Contains the min and the max and the command and state topics for the joint controller.
    """
    def __init__(self, name, command_topic, msg_type, min=0, max=90):
        self.name = name
        self.command_topic = command_topic
        self.msg_type = msg_type
        self.min = min
        self.max = max

class Joint():
    """
    Contains the name, and a controllers list for the joint.
    """
    def __init__(self, name="", joint_controller_list=None):
        self.name = name
        self.controller_list = joint_controller_list

class ExtendedSlider(QFrame):
    """
    This slider displays the current position and the target as well.
    """
    def __init__(self, joint, uiFile, robot_lib, plugin_parent, parent=None):
        QFrame.__init__(self, parent)
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), uiFile)
        loadUi(ui_file, self)

        self.robot_lib = robot_lib

        self.plugin_parent = plugin_parent
        self.joint = joint
        self.current_controller_index = -1
        self.label.setText(joint.name)
        self.current_value = 0

        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setTracking(True)
        self.pos_slider_tracking_behaviour = True
        self.is_selected = False
        self.first_update_done = False

        if len(self.joint.controller_list) > 1:
            self.slider.setMinimum(0)
            self.slider.setMaximum(10)
            self.min_label.setText("Unknown")
            self.max_label.setText("Unknown")
        elif len(self.joint.controller_list) == 1:
            self.current_controller_index = 0
            self.slider.setMinimum(joint.controller_list[self.current_controller_index].min)
            self.slider.setMaximum(joint.controller_list[self.current_controller_index].max)
            self.min_label.setText(str(joint.controller_list[self.current_controller_index].min))
            self.max_label.setText(str(joint.controller_list[self.current_controller_index].max))
        else:
            rospy.logwarn("No controllers defined for this joint")

        self.connect(self.selected, QtCore.SIGNAL('stateChanged(int)'), self.checkbox_click)
        self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)

        self.timer = Qt.QTimer(self)
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.update)
        self.timer.start(200)

    def changeValue(self, value):
        self.target.setText("Tgt: " + str(value))
        self.sendupdate(value)

    def sendupdate(self, value):
        raise NotImplementedError, "Virtual method, please implement."

    def update(self):
        raise NotImplementedError, "Virtual method, please implement."
    
    def refresh(self):
        raise NotImplementedError, "Virtual method, please implement."

    def checkbox_click(self, value):
        self.is_selected = value
    
    def set_slider_behaviour(self):
        raise NotImplementedError, "Virtual method, please implement."
    
    def set_new_slider_behaviour(self, tracking):
        if tracking:
            self.pos_slider_tracking_behaviour = True
        else:
            self.pos_slider_tracking_behaviour = False
        self.set_slider_behaviour()
    

class EtherCATHandSlider(ExtendedSlider):
    """
    Slider for the EtherCAT Hand.
    """
    def __init__(self, joint, uiFile, robot_lib, plugin_parent, parent=None):
        ExtendedSlider.__init__(self, joint, uiFile, robot_lib, plugin_parent, parent)

        self.initialize_controller()

    def initialize_controller(self):
        self.current_controller_index = self.get_current_joint_controller(self.get_current_controllers())

        if (self.current_controller_index == -1):
            raise Exception("No controller found for joint: " + self.joint.name)
        else:
            self.slider.setMinimum(self.joint.controller_list[self.current_controller_index].min)
            self.slider.setMaximum(self.joint.controller_list[self.current_controller_index].max)
            self.min_label.setText(str(self.joint.controller_list[self.current_controller_index].min))
            self.max_label.setText(str(self.joint.controller_list[self.current_controller_index].max))

            self.pub = rospy.Publisher(self.joint.controller_list[self.current_controller_index].command_topic, Float64, latch=True)
            self.set_slider_behaviour()

    def get_current_joint_controller(self, current_controllers):
        for index, joint_controller in enumerate(self.joint.controller_list):
            for controller in current_controllers:
                if (controller.find(self.joint.name.lower() + '_' + joint_controller.name) != -1):
                    return index
        return -1

    def get_current_controllers(self):
        success = True
        list_controllers = rospy.ServiceProxy('pr2_controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
        except rospy.ServiceException:
            success = False

        current_controllers = []

        if success:
            all_loaded_controllers = resp1.controllers
            for state,tmp_contrl in zip(resp1.state,resp1.controllers):
                if state == "running":
                    current_controllers.append(tmp_contrl)
        else:
            rospy.loginfo("Couldn't get list of controllers from pr2_controller_manager/list_controllers service")

        return current_controllers

    def sendupdate(self, value):
        if (self.current_controller_index == -1):
            self.initialize_controller()

        if (self.current_controller_index != -1):
            if (self.joint.controller_list[self.current_controller_index].name == "mixed_position_velocity") or (self.joint.controller_list[self.current_controller_index].name == "position") or (self.joint.controller_list[self.current_controller_index].name == "muscle_position"):
                self.pub.publish(radians(float(value)))
            elif self.joint.controller_list[self.current_controller_index].name == "velocity":
                self.pub.publish(float(value) / 100.0)
            else:
                self.pub.publish(float(value))

    def update(self):
        try:
            if (self.joint.controller_list[self.current_controller_index].name == "mixed_position_velocity") or (self.joint.controller_list[self.current_controller_index].name == "position")  or (self.joint.controller_list[self.current_controller_index].name == "muscle_position"):
                self.current_value = round(degrees(self.robot_lib.get_position(self.joint.name)),1)
            elif (self.joint.controller_list[self.current_controller_index].name == "velocity"):
                self.current_value = round(self.robot_lib.get_velocity(self.joint.name),1)
            elif (self.joint.controller_list[self.current_controller_index].name == "effort"):
                self.current_value = round(self.robot_lib.get_effort(self.joint.name),1)
            self.value.setText("Val: " + str(self.current_value))
            if self.first_update_done == False:
                if (self.joint.controller_list[self.current_controller_index].name == "mixed_position_velocity") or (self.joint.controller_list[self.current_controller_index].name == "position")  or (self.joint.controller_list[self.current_controller_index].name == "muscle_position"):
                    self.slider.setSliderPosition(self.current_value)
                    self.slider.setValue(self.current_value)
                    self.target.setText("Tgt: " + str(self.current_value))
                else:
                    self.target.setText("Tgt: 0.0")
                self.first_update_done = True
        except:
            pass
        
    def refresh(self):
        """
        Refresh the current position of the slider
        """
        if (self.current_controller_index != -1):
            if (self.joint.controller_list[self.current_controller_index].name == "mixed_position_velocity") or (self.joint.controller_list[self.current_controller_index].name == "position") or (self.joint.controller_list[self.current_controller_index].name == "muscle_position"):
                self.slider.setSliderPosition(self.current_value)
                self.slider.setValue(self.current_value)
                self.target.setText("Tgt: " + str(self.current_value))

    def set_slider_behaviour(self):
        """
        Depending on the type of controllers we may want a different behaviour
        """
        if (self.joint.controller_list[self.current_controller_index].name == "mixed_position_velocity") or (self.joint.controller_list[self.current_controller_index].name == "position") or (self.joint.controller_list[self.current_controller_index].name == "muscle_position"):
            if self.pos_slider_tracking_behaviour:
                self.slider.setTracking(True)
            else:
                self.slider.setTracking(False)
        elif (self.joint.controller_list[self.current_controller_index].name == "velocity"):
            self.slider.setTracking(True)
            self.connect(self.slider, QtCore.SIGNAL('sliderReleased()'), self.on_slider_released)
        elif (self.joint.controller_list[self.current_controller_index].name == "effort"):
            self.slider.setTracking(True)
            self.connect(self.slider, QtCore.SIGNAL('sliderReleased()'), self.on_slider_released)

    def on_slider_released(self):
        if (self.joint.controller_list[self.current_controller_index].name == "effort") or (self.joint.controller_list[self.current_controller_index].name == "velocity"):
            self.slider.setSliderPosition(0)
            self.changeValue(0)
        

class CANHandSlider(ExtendedSlider):
    """
    Slider for the CAN Hand.
    """
    def __init__(self, joint, uiFile, robot_lib, plugin_parent, parent=None):
        ExtendedSlider.__init__(self, joint, uiFile, robot_lib, plugin_parent, parent)
        self.set_slider_behaviour()
        self.pub = rospy.Publisher(self.joint.controller_list[self.current_controller_index].command_topic, sendupdate, latch=True)

    def sendupdate(self, value):
        message=[]
        message.append(joint(joint_name=self.joint.name, joint_target=value))
        self.pub.publish(sendupdate(len(message), message))

    def update(self):
        try:
            self.current_value = round(self.robot_lib.valueof(self.joint.name),1)
            self.value.setText("Val: " + str(self.current_value))
            if self.first_update_done == False:
                self.slider.setSliderPosition(self.current_value)
                self.slider.setValue(self.current_value)
                self.target.setText("Tgt: " + str(self.current_value))
                self.first_update_done = True
        except:
            pass
    
    def refresh(self):
        """
        Refresh the current position of the slider
        """
        self.slider.setSliderPosition(self.current_value)
        self.slider.setValue(self.current_value)
        self.target.setText("Tgt: " + str(self.current_value))

    def set_slider_behaviour(self):
        if self.pos_slider_tracking_behaviour:
            self.slider.setTracking(True)
        else:
            self.slider.setTracking(False)


class ArmSlider(ExtendedSlider):
    """
    Slider for the CAN Arm.
    """
    def __init__(self, joint, uiFile, robot_lib, plugin_parent, parent=None):
        ExtendedSlider.__init__(self, joint, uiFile, robot_lib, plugin_parent, parent)
        self.set_slider_behaviour()
        self.pub = rospy.Publisher(self.joint.controller_list[self.current_controller_index].command_topic, sendupdate, latch=True)

    def sendupdate(self, value):
        message=[]
        message.append(joint(joint_name=self.joint.name, joint_target=value))
        self.pub.publish(sendupdate(len(message), message))

    def update(self):
        try:
            self.current_value = round(self.robot_lib.valueof(self.joint.name),1)
            self.value.setText("Val: " + str(self.current_value))
            if self.first_update_done == False:
                self.slider.setSliderPosition(self.current_value)
                self.slider.setValue(self.current_value)
                self.target.setText("Tgt: " + str(self.current_value))
                self.first_update_done = True
        except:
            pass
    
    def refresh(self):
        """
        Refresh the current position of the slider
        """
        self.slider.setSliderPosition(self.current_value)
        self.slider.setValue(self.current_value)
        self.target.setText("Tgt: " + str(self.current_value))
    
    def set_slider_behaviour(self):
        if self.pos_slider_tracking_behaviour:
            self.slider.setTracking(True)
        else:
            self.slider.setTracking(False)

class SelectionSlider(QFrame):
    """
    This slider allows the user to move the selected sliders.
    """
    def __init__(self, name, min, max, uiFile, plugin_parent, parent=None):
        QFrame.__init__(self, parent)
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), uiFile)
        loadUi(ui_file, self)

        self.plugin_parent = plugin_parent
        self.label.setText(name)

        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setTracking(False)
        self.is_selected = False

        self.slider.setMinimum(min)
        self.slider.setMaximum(max)
        self.min_label.setText(str(min))
        self.max_label.setText(str(max))

        self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)

        self.connect(self.selected, QtCore.SIGNAL('stateChanged(int)'), self.checkbox_click)

    def changeValue(self, value):
        raise NotImplementedError, "Virtual method, please implement."

    def checkbox_click(self, value):
        self.is_selected = value
        for slider in self.plugin_parent.sliders:
            if slider.is_selected != value:
                slider.is_selected = value
                slider.selected.setChecked(value)

class EtherCATSelectionSlider(SelectionSlider):
    """
    This slider allows the user to move the selected sliders for an etherCAT hand.
    """
    def __init__(self, name, min, max, uiFile, plugin_parent, parent=None):
        SelectionSlider.__init__(self, name, min, max, uiFile, plugin_parent, parent)
        self.set_slider_behaviour()

    def set_slider_behaviour(self):
        """
        Depending on the type of controllers we may want a different behaviour
        """
        #Currently we set the tracking to true for all the slide types
        self.slider.setTracking(True)
        #If any of the controllers is an effort or velocity controller we will activate the slider released signal detection
        #And set the slider halfway (50) as that is the position of the 0 for effort and velocity controllers
        for slider in self.plugin_parent.sliders:
            if (slider.joint.controller_list[slider.current_controller_index].name == "effort")  or (slider.joint.controller_list[slider.current_controller_index].name == "velocity"):
                self.connect(self.slider, QtCore.SIGNAL('sliderReleased()'), self.on_slider_released)
                self.slider.setSliderPosition(50)
                self.current_value = 50
                self.target.setText("Tgt: " + str(50) + "%")
                break

    def changeValue(self, value):
        #modify the values from the selected sliders.
        for slider in self.plugin_parent.sliders:
            if slider.is_selected:
                temp_value = ((slider.slider.maximum() - slider.slider.minimum()) * float(value) / 100.0) + slider.slider.minimum()
                slider.slider.setSliderPosition(temp_value)
                slider.changeValue(temp_value)

        self.current_value = value
        self.target.setText("Tgt: " + str(value) + "%")

    def on_slider_released(self):
        for slider in self.plugin_parent.sliders:
            if slider.is_selected:
                if (slider.joint.controller_list[slider.current_controller_index].name == "effort") or (slider.joint.controller_list[slider.current_controller_index].name == "velocity"):
                    slider.slider.setSliderPosition(0)
                    slider.changeValue(0)
        self.slider.setSliderPosition(50)
        self.current_value = 50
        self.target.setText("Tgt: " + str(50) + "%")

class CANHandSelectionSlider(SelectionSlider):
    """
    This slider allows the user to move the selected sliders for a ShadowArm.
    """
    def __init__(self, name, min, max, uiFile, robot_lib, plugin_parent, parent=None):
        SelectionSlider.__init__(self, name, min, max, uiFile, plugin_parent, parent)
        self.robot_lib = robot_lib
        self.slider.setTracking(True)

    def changeValue(self, value):
        #modify the values from the selected sliders.
        joint_dict = {}
        for slider in self.plugin_parent.sliders:
            if slider.is_selected:
                temp_value = ((slider.slider.maximum() - slider.slider.minimum()) * float(value) / 100.0) + slider.slider.minimum()
                slider.slider.setSliderPosition(temp_value)
                slider.target.setText("Tgt: " + str(temp_value))
                joint_dict[slider.joint.name] = temp_value

        self.robot_lib.sendupdate_from_dict(joint_dict)
        self.current_value = value
        self.target.setText("Tgt: " + str(value) + "%")

class ArmSelectionSlider(SelectionSlider):
    """
    This slider allows the user to move the selected sliders for a ShadowArm.
    """
    def __init__(self, name, min, max, uiFile, robot_lib, plugin_parent, parent=None):
        SelectionSlider.__init__(self, name, min, max, uiFile, plugin_parent, parent)
        self.robot_lib = robot_lib
        self.slider.setTracking(True)

    def changeValue(self, value):
        #modify the values from the selected sliders.
        joint_dict = {}
        for slider in self.plugin_parent.sliders:
            if slider.is_selected:
                temp_value = ((slider.slider.maximum() - slider.slider.minimum()) * float(value) / 100.0) + slider.slider.minimum()
                slider.slider.setSliderPosition(temp_value)
                slider.target.setText("Tgt: " + str(temp_value))
                joint_dict[slider.joint.name] = temp_value

        self.robot_lib.sendupdate_arm_from_dict(joint_dict)
        self.current_value = value
        self.target.setText("Tgt: " + str(value) + "%")




