#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
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
#

from __future__ import absolute_import
import rospy
import re

from xml.etree import ElementTree as ET
from controller_manager_msgs.srv import ListControllers

from sr_robot_msgs.srv import ForceController, SetEffortControllerGains, SetMixedPositionVelocityPidGains, SetPidGains
from sr_gui_controller_tuner.pid_loader_and_saver import PidLoader, PidSaver


class CtrlSettings(object):

    """
    Parses xml file and reads controller settings
    Creates lists for headers, fingers, motors
    """

    def __init__(self, xml_path, controller_type, joint_prefix):
        self.headers = []

        # open and parses the xml config file
        xml_file = open(xml_path)
        xml_tree = ET.parse(xml_file)
        xml_file.close()

        # read the settings from the xml file
        ctrl_tree = None
        for ctrl in xml_tree.findall("controller"):
            ctrl_name = ctrl.attrib['name']
            if ctrl_name == controller_type:
                ctrl_tree = ctrl
                break

        if ctrl_tree is None:
            rospy.logerr(
                "Couldn't find the settings for the controller " + controller_type)

        # read the headers settings
        xml_headers = ctrl_tree.find("headers")
        for header in xml_headers.findall("item"):
            self.headers.append(header.attrib)

        self.nb_columns = len(self.headers)

        self.hand_item = ["Hand"]
        self.hand_item.extend((self.nb_columns - 1) * [""])

        # read the fingers and the motors from the xml file
        self.fingers = []
        self.motors = []
        all_fingers = xml_tree.find("fingers")
        for finger in all_fingers.findall("finger"):
            finger_row = [finger.attrib['name']]
            finger_row.extend((self.nb_columns - 1) * [""])
            self.fingers.append(finger_row)

            motors_for_finger = []
            for motor in finger.findall("motor"):
                motor_row = ["", joint_prefix + motor.attrib['name']]
                motor_row.extend((self.nb_columns - 1) * [""])
                motors_for_finger.append(motor_row)

            self.motors.append(motors_for_finger)


class SrControllerTunerApp(object):

    """
    Handles loading, saving and setting of controller settings
    """
    CONTROLLER_MANAGER_DETECTION_TIMEOUT = 3.0

    def __init__(self, xml_path):
        self.xml_path = xml_path
        self.all_controller_types = ["Motor Force", "Position", "Velocity",
                                     "Mixed Position/Velocity", "Effort", "Muscle Position"]
        self.pid_loader = PidLoader()

        self.edit_only_mode = False
        self.control_mode = "FORCE"
        # global prefix

        # prefix used in shadow controllers
        self.controller_prefix = "sh_"

        # both prefix and selected_prefix are stored
        # to handle case when gui is started in a namespace, then prefix
        # must be set to empty but selected prefix is still necessary
        # to select which joint_prefix to choose
        self.selected_prefix = ""
        self.prefix = ""
        self.joint_prefix = ""
        # store if one or two loops are running
        self.single_loop = False

        self.namespace = rospy.get_namespace()

    def check_prefix(self):
        """
        Get the prefix (hand and joint)
        Check if it matches selected prefix
        """

        hand_ids = []
        hand_joint_prefixes = []
        # unless incompatible, use the selected prefix by default
        self.prefix = self.selected_prefix
        prefix = self.selected_prefix.rstrip("/")
        # mapping is always in global ns
        if rospy.has_param("/hand/mapping"):
            hand_mapping = rospy.get_param("/hand/mapping")
            for _, value in list(hand_mapping.items()):
                # if prefix matches the mapping, add this hand (empty prefix
                # means both hands)
                if value.startswith(prefix):
                    hand_ids.append(value)
        if len(hand_ids) == 0:
            # no matching hand id to selected prefix, check for namespace
            if prefix in self.namespace:
                rospy.loginfo("Using namespace, no prefix")
                # in this case no prefix is needed
                self.prefix = ""
            else:
                rospy.logwarn("Prefix not matching namespace")
                return False

        if len(hand_ids) > 1:
            # the plugin cannot handle more than one hand
            rospy.logwarn(
                "More than one hand found with prefix :" + prefix + " !\n Not loading controllers")
            return False

        # joint_prefix always in global ns
        if rospy.has_param("/hand/joint_prefix"):
            hand_joint_prefix_mapping = rospy.get_param("/hand/joint_prefix")
            for _, value in list(hand_joint_prefix_mapping.items()):
                # if prefix matches the mapping, add this joint prefix
                if prefix in value:
                    hand_joint_prefixes.append(value)

        if len(hand_joint_prefixes) > 1:
            # the plugin cannot handle more than one hand
            rospy.logwarn(
                "More than one hand found with prefix :" + prefix + " !\n Not loading controllers")
            return False

        if len(hand_joint_prefixes) == 0:
            rospy.logwarn("No hand found with prefix :" + prefix)
            self.joint_prefix = ""
        else:
            self.joint_prefix = hand_joint_prefixes[0]

        rospy.loginfo("using joint_prefix " + self.joint_prefix)
        return True

    def get_ctrls(self):
        """
        Retrieve currently running controllers
        return ["Motor Force", "Position"]
        """
        running_ctrls = []

        # find controller manager in selected namespace

        ctrl_srv_name = self.prefix + 'controller_manager/list_controllers'
        try:
            rospy.wait_for_service(
                ctrl_srv_name, self.CONTROLLER_MANAGER_DETECTION_TIMEOUT)

        except rospy.ROSException:
            # try at root namespace (only in case bimanual setup in a single
            # loop and only if no GUI ns)
            if self.namespace == "/":
                ctrl_srv_name = 'controller_manager/list_controllers'
                try:
                    rospy.wait_for_service(
                        ctrl_srv_name, self.CONTROLLER_MANAGER_DETECTION_TIMEOUT)
                    self.single_loop = True
                    rospy.loginfo("Detected single loop")
                except rospy.ROSException as e:
                    rospy.loginfo(
                        "Controller manager not running: %s" % str(e))
                    rospy.loginfo("Running controller tuner in edit-only mode")
                    return self.set_edit_only(running_ctrls)
            else:
                return self.set_edit_only(running_ctrls)

        # found a controller manager
        controllers = rospy.ServiceProxy(ctrl_srv_name, ListControllers)
        resp = None
        try:
            resp = controllers()
        except rospy.ServiceException as e:
            rospy.logerr("Service did not process request: %s" % str(e))

        running_ctrls.append("Motor Force")
        if resp is not None:
            for controller in resp.controller:
                if controller.state == "running":
                    # find at the specific pattern of the controller
                    splitted = re.split(
                        '[tfmrlw][fhr]j[0-5]_', controller.name)
                    # only consider shadow (prefix sh_) controllers (drop js
                    # ctrl and others)
                    if self.controller_prefix in splitted[0]:
                        ctrl_type_tmp = ""
                        # only consider joint controllers (containing _xxjy_)
                        if len(splitted) >= 2:
                            ctrl_type_tmp = splitted[1]
                        # look at first word of the controller type
                        ctrl_type_tmp_splitted = ctrl_type_tmp.split("_")
                        for defined_ctrl_type in self.all_controller_types:
                            if ctrl_type_tmp_splitted[0].lower() in defined_ctrl_type.lower():
                                running_ctrls.append(defined_ctrl_type)
                                self.edit_only_mode = False
                                return running_ctrls

        rospy.loginfo("No controllers currently running")
        rospy.loginfo("Running controller tuner in edit-only mode")
        del running_ctrls[:]
        return self.set_edit_only(running_ctrls)

    def set_edit_only(self, running_ctrls):
        """
        Sets all the controllers to defined type for edit-only mode
        """
        self.edit_only_mode = True
        # In edit_only_mode all the controllers are available for editing
        for defined_ctrl_type in self.all_controller_types:
            running_ctrls.append(defined_ctrl_type)
        return running_ctrls

    def refresh_control_mode(self):
        """
        Effectively change control mode on the realtime loop
        """
        self.control_mode = rospy.get_param(
            'sr_hand_robot/' + self.prefix + 'default_control_mode', 'FORCE')

    def get_controller_settings(self, controller_type):
        """
        Parses a file containing the controller settings
        and their min and max values, and returns them.
        """
        ctrl_settings = CtrlSettings(
            self.xml_path, controller_type, self.joint_prefix)

        return ctrl_settings

    def load_parameters(self, controller_type, joint_name):
        """
        Load the parameters from the yaml file.
        """
        param_name = ""
        prefix = self.prefix if self.single_loop is not True else ""
        if controller_type == "Motor Force":
            # currently the motor_board topics use non-prefixed joint names
            # no matter if single or dual loops there is always a prefix for
            # motors
            param_name = self.prefix + \
                joint_name.strip(
                    self.joint_prefix.rstrip("/") + "_").lower() + "/pid"
        elif controller_type == "Position":
            param_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_position_controller/pid"
        elif controller_type == "Muscle Position":
            param_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_muscle_position_controller/pid"
        elif controller_type == "Velocity":
            param_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_velocity_controller/pid"
        elif controller_type == "Mixed Position/Velocity":
            param_name = [prefix + self.controller_prefix + joint_name.lower() +
                          "_mixed_position_velocity_controller/position_pid",
                          prefix + self.controller_prefix + joint_name.lower() +
                          "_mixed_position_velocity_controller/velocity_pid"]
        elif controller_type == "Effort":
            param_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_effort_controller"

        return self.pid_loader.get_settings(param_name)

    def set_controller(self, joint_name, controller_type, controller_settings):
        """
        Sets the controller settings calling the proper service with the correct syntax for controller type.
        """
        pid_service = None
        service_name = ""
        prefix = self.prefix if self.single_loop is not True else ""

        if controller_type == "Motor Force":
            # /sr_hand_robot/change_force_PID_FFJ0
            # currently use non-prefixed joint names but adds prefix in the middle
            # no matter if single or dual loops there is always a prefix for
            # motors
            service_name = "sr_hand_robot/" + self.prefix + \
                "change_force_PID_" + joint_name[-4:].upper()
            pid_service = rospy.ServiceProxy(service_name, ForceController)

        elif controller_type == "Position":
            # /sh_ffj3_position_controller/set_gains
            service_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_position_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif controller_type == "Muscle Position":
            # /sh_ffj3_position_controller/set_gains
            service_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_muscle_position_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif controller_type == "Velocity":
            # /sh_ffj3_velocity_controller/set_gains
            service_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_velocity_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif controller_type == "Mixed Position/Velocity":
            # /sh_ffj3_mixed_position_velocity_controller/set_gains
            service_name = prefix + self.controller_prefix + \
                joint_name.lower() + \
                "_mixed_position_velocity_controller/set_gains"
            pid_service = rospy.ServiceProxy(
                service_name, SetMixedPositionVelocityPidGains)

        elif controller_type == "Effort":
            # /sh_ffj3_effort_controller/set_gains
            service_name = prefix + self.controller_prefix + \
                joint_name.lower() + "_effort_controller/set_gains"
            pid_service = rospy.ServiceProxy(
                service_name, SetEffortControllerGains)

        else:
            rospy.logerr(
                "", controller_type, " is not a recognized controller type.")

        contrlr_settings_converted = {}
        for param in list(controller_settings.items()):
            contrlr_settings_converted[param[0]] = float(param[1])

        if controller_type == "Motor Force":
            try:
                for setting in ["torque_limit", "torque_limiter_gain"]:
                    if setting not in contrlr_settings_converted:
                        contrlr_settings_converted[setting] = 0
                pid_service(int(contrlr_settings_converted["max_pwm"]),
                            int(contrlr_settings_converted["sgleftref"]),
                            int(contrlr_settings_converted["sgrightref"]),
                            int(contrlr_settings_converted["f"]),
                            int(contrlr_settings_converted["p"]), int(
                                contrlr_settings_converted["i"]),
                            int(contrlr_settings_converted["d"]), int(
                                contrlr_settings_converted["imax"]),
                            int(contrlr_settings_converted["deadband"]),
                            int(contrlr_settings_converted["sign"]),
                            int(contrlr_settings_converted["torque_limit"]),
                            int(contrlr_settings_converted["torque_limiter_gain"]))
            except rospy.ServiceException:
                return False

        elif controller_type == "Position":
            try:
                pid_service(
                    float(contrlr_settings_converted["p"]), float(
                        contrlr_settings_converted["i"]),
                    float(contrlr_settings_converted["d"]), float(
                        contrlr_settings_converted["i_clamp"]),
                    float(contrlr_settings_converted["max_force"]), float(
                        contrlr_settings_converted[
                            "position_deadband"]),
                    int(contrlr_settings_converted["friction_deadband"]))
            except rospy.ServiceException:
                return False

        elif controller_type == "Muscle Position":
            try:
                pid_service(
                    float(contrlr_settings_converted["p"]), float(
                        contrlr_settings_converted["i"]),
                    float(contrlr_settings_converted["d"]), float(
                        contrlr_settings_converted["i_clamp"]),
                    float(contrlr_settings_converted["max_force"]), float(
                        contrlr_settings_converted[
                            "position_deadband"]),
                    int(contrlr_settings_converted["friction_deadband"]))
            except rospy.ServiceException:
                return False

        elif controller_type == "Velocity":
            try:
                pid_service(
                    float(contrlr_settings_converted["p"]), float(
                        contrlr_settings_converted["i"]),
                    float(contrlr_settings_converted["d"]), float(
                        contrlr_settings_converted["i_clamp"]),
                    float(contrlr_settings_converted["max_force"]), float(
                        contrlr_settings_converted[
                            "velocity_deadband"]),
                    int(contrlr_settings_converted["friction_deadband"]))
            except rospy.ServiceException:
                return False

        elif controller_type == "Mixed Position/Velocity":
            try:
                pid_service(
                    float(contrlr_settings_converted["pos/p"]), float(
                        contrlr_settings_converted["pos/i"]),
                    float(contrlr_settings_converted["pos/d"]), float(
                        contrlr_settings_converted["pos/i_clamp"]),
                    float(contrlr_settings_converted["pos/min_velocity"]), float(
                        contrlr_settings_converted[
                            "pos/max_velocity"]),
                    float(contrlr_settings_converted[
                          "pos/position_deadband"]),
                    float(contrlr_settings_converted["vel/p"]), float(
                        contrlr_settings_converted["vel/i"]),
                    float(contrlr_settings_converted["vel/d"]), float(
                        contrlr_settings_converted["vel/i_clamp"]),
                    float(contrlr_settings_converted["vel/max_force"]),
                    int(contrlr_settings_converted["vel/friction_deadband"]))
            except rospy.ServiceException:
                return False

        elif controller_type == "Effort":
            try:
                pid_service(int(contrlr_settings_converted["max_force"]), int(
                    contrlr_settings_converted["friction_deadband"]))
            except rospy.ServiceException:
                return False
        else:
            rospy.logerr(
                "", controller_type, " is not a recognized controller type.")
            return False
        return True

    def save_controller(self, joint_name, controller_type, controller_settings, filename):
        """
        Saves the controller settings calling the proper service with the correct syntax for controller type
        """
        param_name = []
        prefix = self.prefix if self.single_loop is not True else ""
        if controller_type == "Motor Force":
            param_name = ["" + joint_name[-4:].lower(), "pid"]
        elif controller_type == "Position":
            param_name = [prefix + self.controller_prefix +
                          joint_name.lower() + "_position_controller", "pid"]
        elif controller_type == "Muscle Position":
            param_name = [prefix + self.controller_prefix +
                          joint_name.lower() + "_muscle_position_controller", "pid"]
        elif controller_type == "Velocity":
            param_name = [prefix + self.controller_prefix +
                          joint_name.lower() + "_velocity_controller", "pid"]
        elif controller_type == "Mixed Position/Velocity":
            param_name = [prefix + self.controller_prefix +
                          joint_name.lower() + "_mixed_position_velocity_controller", "pid"]
        elif controller_type == "Effort":
            param_name = [prefix + self.controller_prefix +
                          joint_name.lower() + "_effort_controller"]
        pid_saver = PidSaver(filename)
        pid_saver.save_settings(param_name, controller_settings)
