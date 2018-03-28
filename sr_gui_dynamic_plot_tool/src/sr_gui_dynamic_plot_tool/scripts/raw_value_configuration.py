#!/usr/bin/env python
#
# Copyright 2018 Shadow Robot Company Ltd.

import os
import rospkg
import rospy
import sys
from sr_gui_dynamic_plot_tool.dynamic_plot_tool import CreatePlotConfigurations
from collections import namedtuple
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers

TopicStruct = namedtuple('TopicStructure', "topic_name topic_field msg_type time_receipt")


class SrAddInterfaceEntries():
    def __init__(self):
        self._widget_choice = {}
        self._joint_state_msg = JointState()
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)

    def define_interface_setting(self):
        """
        Add here the entries for your interface.
        Format - append name of item and number of items.
        By default buttons for hand, fingers and joints will be added
        """
        self._widget_choice['Configuration'] = ["Raw_Encoder_Torque", "Raw_Encoder_Position",
                                                "Position_Control", "Torque_Control"]
        return self._widget_choice

    def define_plot_settings(self, choices):
        """
        Add here your plot settings
        @param choices - list containing user selection from the GUI
        """
        #TODO(@Giuseppe): search for hand in choices to avoid order confusion
        hand_choice = choices[0]
        finger_choice = choices[1]
        joint_choice = choices[2]
        configuration_choice = choices[3]

        joint_position_choice = int(joint_choice[-1:]) + int(joint_choice[-1:])
        joint_torque_choice = joint_position_choice + 1
        choice_argument = hand_choice + "_" + finger_choice + joint_choice

        controller_type = self._check_loaded_controllers()

        joint_state_selection = self._get_joint_state_topic(choice_argument)

        # Position Control Topic
        if controller_type == "trajectory":
            position_control_topic = self.create_position_control_topic(hand_choice, joint_state_selection, time_receipt=False)
            position_control_time_receipt = self.create_position_control_topic(hand_choice, joint_state_selection, time_receipt=True)
        elif controller_type == "position":
            pass

        # Joint State topic
        joint_state_position_topic = self.create_joint_state_topic(joint_state_selection, "position", time_receipt=False)
        joint_state_effort_topic = self.create_joint_state_topic(joint_state_selection, "effort", time_receipt=False)
        joint_state_time_receipt = self.create_joint_state_topic(joint_state_selection, "position", time_receipt=True)

        # Raw Encoder data topic
        raw_encoder_position_topic = self.create_raw_data_topic(hand_choice, finger_choice, joint_position_choice, time_receipt=False)
        raw_encoder_torque_topic = self.create_raw_data_topic(hand_choice, finger_choice, joint_torque_choice, time_receipt=False)
        raw_encoder_time_receipt = self.create_raw_data_topic(hand_choice, finger_choice, "0", time_receipt=True)

        # Command Torque topic
        commanded_torque_topic = self.create_torque_command_topic(hand_choice, finger_choice, joint_choice, time_receipt=False)
        commanded_torque_time_receipt = self.create_torque_command_topic(hand_choice, finger_choice, joint_choice, time_receipt=True)


        # Create configuration xml file.
        plots = CreatePlotConfigurations(1, 1, "base_configuration.xml")
        plots_list = plots._plots

        # Add topic to plot to the corresponding plot
        if configuration_choice == "Raw_Encoder_Position":
            plots_list[0].set_title_and_frame_rate("{}_{}{} Position Encoder Raw Data".format(hand_choice,
                                                                                              finger_choice,
                                                                                              joint_choice), 30)
            plots_list[0].add_curve(raw_encoder_time_receipt, raw_encoder_position_topic, 0)
        elif configuration_choice == "Raw_Encoder_Torque":
            plots_list[0].set_title_and_frame_rate("{}_{}{} Torque Encoder Raw Data".format(hand_choice,
                                                                                            finger_choice,
                                                                                            joint_choice), 30)
            plots_list[0].add_curve(raw_encoder_time_receipt, raw_encoder_torque_topic, 0)
        elif configuration_choice == "Position_Control":
            plots_list[0].set_title_and_frame_rate("{}_{}{} CommandedPos vs ActualPos".format(hand_choice,
                                                                                                    finger_choice,
                                                                                                    joint_choice), 30)
            plots_list[0].add_curve(position_control_time_receipt, position_control_topic, 0)
            plots_list[0].add_curve(joint_state_time_receipt, joint_state_position_topic, 1)
        elif configuration_choice == "Torque_Control":
            plots_list[0].set_title_and_frame_rate("{}_{}{} CommandedTorque vs ActualTorque".format(hand_choice,
                                                                                                    finger_choice,
                                                                                                    joint_choice), 30)
            plots_list[0].add_curve(commanded_torque_time_receipt, joint_state_effort_topic, 0)
            plots_list[0].add_curve(joint_state_time_receipt, commanded_torque_topic, 1)
        else:
            rospy.logerr("No configuration selected")

    def _get_joint_state_topic(self, selected_joint_name):
        for index, name in enumerate(self._joint_state_msg.name):
            if name == selected_joint_name:
                return index

    def _joint_state_callback(self, data):
        self._joint_state_msg = data

    def _check_loaded_controllers(self):
        list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
            for controller in resp1.controller:
                if controller.type == "effort_controllers/JointTrajectoryController":
                    return "trajectory"
                elif controller.type == "effort_controller/JointPositionController":
                    return "position"
        except rospy.ServiceException:
            rospy.logerr("Could not get any controller")

    def create_position_control_topic(self, hand_choice, joint_selected, time_receipt):
        PositionControlTopic = TopicStruct(topic_name="/{}_trajectory_controller/follow_joint_trajectory/feedback".format(hand_choice),
                                           topic_field = "feedback/desired/positions/{}".format(joint_selected),
                                           msg_type="control_msgs/FollowJointTrajectoryActionFeedback",
                                           time_receipt=time_receipt)
        return PositionControlTopic

    def create_joint_state_topic(self, joint_selected, topic_field, time_receipt):
        JointPositionTopic = TopicStruct(topic_name="/joint_states",
                                         topic_field="{}/{}".format(topic_field, joint_selected),
                                         msg_type="sensor_msgs/JointState",
                                         time_receipt=time_receipt)
        return JointPositionTopic

    def create_raw_data_topic(self, hand_choice, finger_choice, joint_choice, time_receipt):
        EncoderPosTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(hand_choice, finger_choice),
                                      topic_field="data/0/int16s_values/{}".format(joint_choice),
                                      msg_type="fh_msgs/FhState",
                                      time_receipt=time_receipt)
        return EncoderPosTopic
    
    def create_torque_command_topic(self, hand_choice, finger_choice, joint_choice, time_receipt):
        CommandTorqueTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(hand_choice, finger_choice),
                                         topic_field="data/0/int16s_values/{}".format(joint_choice[-1:]),
                                         msg_type="fh_msgs/FhCommand",
                                         time_receipt=time_receipt)
        return CommandTorqueTopic
