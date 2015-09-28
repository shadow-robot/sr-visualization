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
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sr_gui_joint_slider.sliders import JointController, Joint, EtherCATHandSlider
from sr_gui_joint_slider.sliders import EtherCATHandTrajectorySlider, EtherCATSelectionSlider


class SrGuiJointSlider(Plugin):

    """
    A rosgui plugin to change the position of the different joints
    """

    # For each controller type this defines the category of controller it belongs to
    # (position, velocity, effort, position_trajectory)
    # and the msg type of the controller state topic
    controller_state_types = {
        "sr_mechanism_controllers/SrhJointPositionController": ("position", JointControllerState),
        "sr_mechanism_controllers/SrhEffortJointController": ("effort", JointControllerState),
        "sr_mechanism_controllers/SrhJointVelocityController": ("velocity", JointControllerState),
        "sr_mechanism_controllers/SrhMixedPositionVelocityJointController": ("position", SrJointControllerState),
        "sr_mechanism_controllers/SrhMuscleJointPositionController": ("position", JointMusclePositionControllerState),
        "position_controllers/JointTrajectoryController": ("position_trajectory", JointTrajectoryControllerState),
        "effort_controllers/JointTrajectoryController": ("position_trajectory", JointTrajectoryControllerState)}

    def __init__(self, context):
        super(SrGuiJointSlider, self).__init__(context)
        self.setObjectName('SrGuiJointSlider')

        self._robot_description_xml_root = None

        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_joint_slider'), 'uis', 'SrJointSlider.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('SrJointSliderUi')
        context.add_widget(self._widget)

        self.joints = []

        self.sliders = []
        self.selection_slider = None

        # to be used by trajectory controller sliders
        self.trajectory_state_sub = []
        # self.trajectory_state = []
        self.trajectory_state_slider_cb = []
        self.trajectory_pub = []
        self.trajectory_target = []

        self._widget.reloadButton.pressed.connect(
            self.on_reload_button_cicked_)
        self._widget.refreshButton.pressed.connect(
            self.on_refresh_button_cicked_)
        self._widget.sliderReleaseCheckBox.stateChanged.connect(
            self.on_slider_release_checkbox_clicked_)

        self._widget.reloadButton.setEnabled(True)
        self.on_reload_button_cicked_()

    def _unregister(self):
        pass

    def shutdown_plugin(self):
        self._unregister()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass

    def on_robot_type_changed_(self):
        pass

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

        self._widget.sliderReleaseCheckBox.setCheckState(Qt.Unchecked)

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

    def load_new_sliders_(self):
        """
        Create the new slider widgets
        Load the new slider
        Put the slider in the list
        """
        self.sliders = list()
        for joint in self.joints:
            slider = None
            slider_ui_file = os.path.join(
                rospkg.RosPack().get_path('sr_gui_joint_slider'), 'uis', 'Slider.ui')

            try:
                if joint.controller.controller_category == "position_trajectory":
                    slider = EtherCATHandTrajectorySlider(
                        joint, slider_ui_file, self, self._widget.scrollAreaWidgetContents)
                else:
                    slider = EtherCATHandSlider(
                        joint, slider_ui_file, self, self._widget.scrollAreaWidgetContents)
            except Exception, e:
                rospy.loginfo(e)

            if slider is not None:
                slider.setMaximumWidth(100)
                # Load the new slider
                self._widget.horizontalLayout.addWidget(slider)
                # Put the slider in the list
                self.sliders.append(slider)

        # Create the slider to move all the selected joint sliders
        selection_slider_ui_file = os.path.join(
            rospkg.RosPack().get_path('sr_gui_joint_slider'), 'uis', 'SelectionSlider.ui')
        self.selection_slider = EtherCATSelectionSlider(
            "Change sel.", 0, 100, selection_slider_ui_file, self, self._widget.scrollAreaWidgetContents)

        self.selection_slider.setMaximumWidth(100)
        self._widget.horizontalLayout.addWidget(self.selection_slider)

    def get_current_controllers(self):
        """
        @return: list of current controllers with associated data
        """
        success = True
        list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            return [c for c in resp1.controller if c.state == "running"]
        else:
            rospy.loginfo(
                "Couldn't get list of controllers from controller_manager/list_controllers service")
            return []

    def _load_robot_description(self):
        """
        Load the description from the param named in the edit as an ET element.
        Sets self._robot_description_xml_root to the element.
        """
        name = self._widget.robot_description_edit.text()
        self._robot_description_xml_root = None
        try:
            xml = rospy.get_param(name)
            self._robot_description_xml_root = ET.fromstring(xml)
        except KeyError as e:
            rospy.logerr(
                "Failed to get robot description from param %s : %s" % (name, e))
            return
        except:
            raise

    def _get_joint_min_max_vel(self, jname):
        """Get the min and max from the robot description for a given joint."""
        root = self._robot_description_xml_root
        if root is not None:
            limit = root.findall(".//joint[@name='" + jname + "']/limit")
            if limit is None or len(limit) == 0:
                # Handles upper case joint names in the model. e.g. the E1
                # shadowhand
                limit = root.findall(
                    ".//joint[@name='" + jname.upper() + "']/limit")
            if limit is not None and len(limit) > 0:
                return (float(limit[0].attrib['lower']),
                        float(limit[0].attrib['upper']),
                        float(limit[0].attrib['velocity']))
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
        trajectory_ctrl_joint_names = []
        self.trajectory_target = []
        self.trajectory_state_sub = []
        # self.trajectory_state = []
        self.trajectory_state_slider_cb = []
        self.trajectory_pub = []

        for controller in controllers:
            if controller.type == "position_controllers/JointTrajectoryController":
                for j_name in controller.resources:
                    trajectory_ctrl_joint_names.append(j_name)

        for controller in controllers:
            if rospy.has_param(controller.name):
                ctrl_params = rospy.get_param(controller.name)
                controller_type = ctrl_params["type"]
                if controller_type in self.controller_state_types:
                    controller_state_type = self.controller_state_types[
                        controller_type][1]
                    controller_category = self.controller_state_types[
                        controller_type][0]

                    if controller_category == "position_trajectory":
                        # for a trajectory controller we will load a slider for every resource it manages
                        self.trajectory_target.append(JointTrajectory())
                        self.trajectory_state_sub.append(
                            rospy.Subscriber(controller.name + "/state", controller_state_type,
                                             self._trajectory_state_cb,
                                             callback_args=len(self.trajectory_state_sub)))

                        self.trajectory_state_slider_cb.append([])
                        self.trajectory_pub.append(
                            rospy.Publisher(controller.name + "/command", JointTrajectory, queue_size=1, latch=True))
                        for j_name in controller.resources:
                            joint_controller = JointController(
                                controller.name, controller_type, controller_state_type, controller_category,
                                self.trajectory_state_slider_cb[
                                    len(self.trajectory_state_slider_cb) - 1],
                                self.trajectory_pub[
                                    len(self.trajectory_pub) - 1],
                                self.trajectory_target[len(self.trajectory_target) - 1])
                            rospy.loginfo(
                                "controller category: %s", controller_category)

                            if self._widget.joint_name_filter_edit.text() not in j_name:
                                continue

                            min, max, vel = self._get_joint_min_max_vel_special(
                                j_name)
                            joint = Joint(
                                j_name, min, max, vel, joint_controller)
                            joints.append(joint)
                    else:
                        joint_name = ctrl_params["joint"]
                        if joint_name in trajectory_ctrl_joint_names:
                            # These joints are controlled by the trajectory controller
                            continue

                        if "J0" in joint_name:  # xxJ0 are controlled by the by the trajectory controller xxJ1 and xxJ2
                            jname1 = joint_name.replace("J0", "J1")
                            jname2 = joint_name.replace("J0", "J2")
                            if jname1 in trajectory_ctrl_joint_names \
                                    and jname2 in trajectory_ctrl_joint_names:
                                continue

                        joint_controller = JointController(
                            controller.name, controller_type, controller_state_type, controller_category)
                        rospy.loginfo(
                            "controller category: %s", controller_category)

                        if self._widget.joint_name_filter_edit.text() not in joint_name:
                            continue

                        min, max, vel = self._get_joint_min_max_vel_special(
                            joint_name)
                        joint = Joint(
                            joint_name, min, max, vel, joint_controller)
                        joints.append(joint)
                else:
                    rospy.logwarn(
                        "Controller %s of type %s not supported", controller.name, controller_type)
                    continue
            else:
                rospy.logwarn(
                    "Parameters for controller %s not found", controller.name)
                continue

        return joints

    def _trajectory_state_cb(self, msg, index):
        if not self.trajectory_target[index].joint_names:  # Initialize the targets with the current position
            self.trajectory_target[index].joint_names = msg.joint_names
            point = JointTrajectoryPoint()
            point.positions = list(msg.actual.positions)  # This is a list for some reason? Should be tuple..
            point.velocities = [0] * len(msg.joint_names)
            point.time_from_start = rospy.Duration.from_sec(0.005)
            self.trajectory_target[index].points = [point]

        for cb in self.trajectory_state_slider_cb[index]:  # call the callbacks of the sliders in the list
            cb(msg)
