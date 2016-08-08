#!/usr/bin/env python
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
#

import rospy

from xml.etree import ElementTree as ET
from controller_manager_msgs.srv import ListControllers

from sr_robot_msgs.srv import ForceController, SetEffortControllerGains, SetMixedPositionVelocityPidGains, SetPidGains
from sr_gui_controller_tuner.pid_loader_and_saver import PidLoader, PidSaver
import unicodedata

class CtrlSettings(object):
    """
    Parses xml file and reads controller settings
    Creates lists for headers, fingers, motors
    """

    def __init__(self, xml_path, controller_type):
        self.headers = []

        #open and parses the xml config file
        xml_file = open(xml_path)
        xml_tree = ET.parse(xml_file)
        xml_file.close()

        #read the settings from the xml file
        ctrl_tree = None
        for ctrl in xml_tree.findall("controller"):
            ctrl_name = ctrl.attrib['name']
            if ctrl_name == controller_type:
                ctrl_tree = ctrl
                break

        if ctrl_tree == None:
            rospy.logerr("Couldn't find the settings for the controller " + controller_type)

        #read the headers settings
        xml_headers = ctrl.find("headers")
        for header in xml_headers.findall("item"):
            self.headers.append( header.attrib )

        self.nb_columns = len(self.headers)

        self.hand_item = ["Hand"]
        for i in range(0, self.nb_columns - 1):
            self.hand_item.append("")

        #read the fingers and the motors from the xml file
        self.fingers = []
        self.motors = []
        all_fingers = xml_tree.find("fingers")
        for finger in all_fingers.findall("finger"):
            finger_row = [ finger.attrib['name'] ]
            for i in range(0, self.nb_columns - 1):
                finger_row.append("")
            self.fingers.append( finger_row )

            motors_for_finger = []
            for motor in finger.findall("motor"):
                motor_row = [ "", motor.attrib['name'] ]
                for i in range(0, self.nb_columns - 2):
                    motor_row.append("")
                motors_for_finger.append( motor_row )

            self.motors.append( motors_for_finger )


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
        

    def get_ctrls(self):
        """
        Retrieve currentlly running controllers
        return ["Motor Force", "Position"]
        """
        
        running_ctrls = []

        try:
            rospy.wait_for_service('controller_manager/list_controllers', self.CONTROLLER_MANAGER_DETECTION_TIMEOUT)
        
            controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
            resp = None
            try:
                resp = controllers()
            except rospy.ServiceException, e:
                rospy.logerr( "Service did not process request: %s"%str(e) )
            
            running_ctrls.append("Motor Force")
            if resp != None:
                for controller in resp.controller:
                    if controller.state == "running":
                        splitted = controller.name.split("_")
                        ctrl_type_tmp = splitted[2]
                        for defined_ctrl_type in self.all_controller_types:
                            if ctrl_type_tmp.lower() in defined_ctrl_type.lower():
                                running_ctrls.append(defined_ctrl_type)
                                self.edit_only_mode = False
                                return running_ctrls
            rospy.loginfo( "No controllers currently running" )
            rospy.loginfo( "Running controller tuner in edit-only mode" )
            self.edit_only_mode = True
            del running_ctrls[:]
            #In edit_only_mode all the controllers are available for editing
            for defined_ctrl_type in self.all_controller_types:
                running_ctrls.append(defined_ctrl_type)
        except rospy.ROSException, e:
            rospy.loginfo( "Controller manager not running: %s"%str(e) )
            rospy.loginfo( "Running controller tuner in edit-only mode" )
            self.edit_only_mode = True
            #In edit_only_mode all the controllers are available for editing
            for defined_ctrl_type in self.all_controller_types:
                running_ctrls.append(defined_ctrl_type)
        
        return running_ctrls
    
    def refresh_control_mode(self):
        self.control_mode = rospy.get_param('realtime_loop/default_control_mode', 'FORCE')

    def get_controller_settings( self, controller_type ):
        """
        Parses a file containing the controller settings
        and their min and max values, and returns them.
        """
        ctrl_settings = CtrlSettings(self.xml_path, controller_type)

        return ctrl_settings

    def load_parameters(self, controller_type, joint_name):
        """
        Load the parameters from the yaml file.
        """
        param_name = ""
        if controller_type == "Motor Force":
            param_name = joint_name.lower() +"/pid"
        elif controller_type == "Position":
            param_name =  "sh_"+ joint_name.lower()+"_position_controller/pid"
        elif controller_type == "Muscle Position":
            param_name =  "sh_"+ joint_name.lower()+"_muscle_position_controller/pid"
        elif controller_type == "Velocity":
            param_name =  "sh_"+ joint_name.lower()+"_velocity_controller/pid"
        elif controller_type == "Mixed Position/Velocity":
            param_name = ["sh_"+ joint_name.lower()+"_mixed_position_velocity_controller/position_pid",
                          "sh_"+ joint_name.lower()+"_mixed_position_velocity_controller/velocity_pid" ]
        elif controller_type == "Effort":
            param_name =  "sh_"+ joint_name.lower()+"_effort_controller"

        return self.pid_loader.get_settings( param_name )

    def set_controller(self, joint_name, controller_type, controller_settings):
        """
        Sets the controller settings calling the proper service with the correct syntax for controller type.
        """
        pid_service = None
        service_name = ""
        if controller_type == "Motor Force":
            #/realtime_loop/change_force_PID_FFJ0
            service_name =  "realtime_loop/change_force_PID_"+joint_name.upper()
            pid_service = rospy.ServiceProxy(service_name, ForceController)

        elif controller_type == "Position":
            #/sh_ffj3_position_controller/set_gains
            service_name =  "sh_"+joint_name.lower()+"_position_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif controller_type == "Muscle Position":
            #/sh_ffj3_position_controller/set_gains
            service_name =  "sh_"+joint_name.lower()+"_muscle_position_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif controller_type == "Velocity":
            #/sh_ffj3_velocity_controller/set_gains
            service_name =  "sh_"+joint_name.lower()+"_velocity_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif controller_type == "Mixed Position/Velocity":
            #/sh_ffj3_mixed_position_velocity_controller/set_gains
            service_name =  "sh_"+joint_name.lower()+"_mixed_position_velocity_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetMixedPositionVelocityPidGains)

        elif controller_type == "Effort":
            #/sh_ffj3_effort_controller/set_gains
            service_name =  "sh_"+joint_name.lower()+"_effort_controller/set_gains"
            pid_service = rospy.ServiceProxy(service_name, SetEffortControllerGains)

        else:
            rospy.logerr( "", controller_type, " is not a recognized controller type." )

        contrlr_settings_converted = {}
        for param in controller_settings.items():
            contrlr_settings_converted[ param[0] ] = float(param[1])

        if controller_type == "Motor Force":
            try:
                pid_service(int(contrlr_settings_converted["max_pwm"]),
                            int(contrlr_settings_converted["sgleftref"]),
                            int(contrlr_settings_converted["sgrightref"]),
                            int(contrlr_settings_converted["f"]),
                            int(contrlr_settings_converted["p"]), int(contrlr_settings_converted["i"]),
                            int(contrlr_settings_converted["d"]), int(contrlr_settings_converted["imax"]),
                            int(contrlr_settings_converted["deadband"]), int(contrlr_settings_converted["sign"]) )
            except:
                return False

        elif controller_type == "Position":
            try:
                pid_service(float(contrlr_settings_converted["p"]), float(contrlr_settings_converted["i"]),
                            float(contrlr_settings_converted["d"]), float(contrlr_settings_converted["i_clamp"]),
                            float(contrlr_settings_converted["max_force"]), float(contrlr_settings_converted["position_deadband"]),
                            int(contrlr_settings_converted["friction_deadband"]) )
            except:
                return False

        elif controller_type == "Muscle Position":
            try:
                pid_service(float(contrlr_settings_converted["p"]), float(contrlr_settings_converted["i"]),
                            float(contrlr_settings_converted["d"]), float(contrlr_settings_converted["i_clamp"]),
                            float(contrlr_settings_converted["max_force"]), float(contrlr_settings_converted["position_deadband"]),
                            int(contrlr_settings_converted["friction_deadband"]) )
            except:
                return False

        elif controller_type == "Velocity":
            try:
                pid_service(float(contrlr_settings_converted["p"]), float(contrlr_settings_converted["i"]),
                            float(contrlr_settings_converted["d"]), float(contrlr_settings_converted["i_clamp"]),
                            float(contrlr_settings_converted["max_force"]), float(contrlr_settings_converted["velocity_deadband"]),
                            int(contrlr_settings_converted["friction_deadband"]) )
            except:
                return False

        elif controller_type == "Mixed Position/Velocity":
            try:
                pid_service(float(contrlr_settings_converted["pos/p"]), float(contrlr_settings_converted["pos/i"]),
                            float(contrlr_settings_converted["pos/d"]), float(contrlr_settings_converted["pos/i_clamp"]),
                            float(contrlr_settings_converted["pos/min_velocity"]), float(contrlr_settings_converted["pos/max_velocity"]),
                            float(contrlr_settings_converted["pos/position_deadband"]),
                            float(contrlr_settings_converted["vel/p"]), float(contrlr_settings_converted["vel/i"]),
                            float(contrlr_settings_converted["vel/d"]), float(contrlr_settings_converted["vel/i_clamp"]),
                            float(contrlr_settings_converted["vel/max_force"]),
                            int(contrlr_settings_converted["vel/friction_deadband"]) )
            except:
                return False

        elif controller_type == "Effort":
            try:
                pid_service(int(contrlr_settings_converted["max_force"]), int(contrlr_settings_converted["friction_deadband"]) )
            except:
                return False
        else:
            rospy.logerr( "", controller_type, " is not a recognized controller type." )
            return False
        return True


    def save_controller(self, joint_name, controller_type, controller_settings, filename):
        """
        Saves the controller settings calling the proper service with the correct syntax for controller type
        """
        param_name = []
        if controller_type == "Motor Force":
            param_name = [""+joint_name.lower() ,"pid"]
        elif controller_type == "Position":
            param_name =  ["sh_"+joint_name.lower()+"_position_controller" , "pid"]
        elif controller_type == "Muscle Position":
            param_name =  ["sh_"+joint_name.lower()+"_muscle_position_controller" , "pid"]
        elif controller_type == "Velocity":
            param_name =  ["sh_"+joint_name.lower()+"_velocity_controller" , "pid"]
        elif controller_type == "Mixed Position/Velocity":
            param_name =  ["sh_"+joint_name.lower()+"_mixed_position_velocity_controller" , "pid"]
        elif controller_type == "Effort":
            param_name =  ["sh_"+joint_name.lower()+"_effort_controller"]

        pid_saver = PidSaver(filename)
        pid_saver.save_settings(param_name, controller_settings)