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

        # Joint State topic
        joint_state_selection = self._get_joint_state_topic(choice_argument)

        JointPositionTopic = TopicStruct(topic_name="/joint_state",
                                         topic_field="/position[{}]".format(joint_state_selection),
                                         msg_type="sensor_msgs/JointState",
                                         time_receipt=False)

        JointEffortTopic = TopicStruct(topic_name="/joint_state",
                                       topic_field="/effort[{}]".format(joint_state_selection),
                                       msg_type="sensor_msgs/JointState",
                                       time_receipt=False)

        # Encoder data topic
        EncoderPosTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(hand_choice, finger_choice),
                                      topic_field="data/0/int16s_values/{}".format(joint_position_choice),
                                      msg_type="fh_msgs/FhState",
                                      time_receipt=False)

        EncoderTorTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(hand_choice, finger_choice),
                                      topic_field="data/0/int16s_values/{}".format(joint_torque_choice),
                                      msg_type="fh_msgs/FhState",
                                      time_receipt=False)

        EncoderTimeTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(hand_choice, finger_choice),
                                       topic_field="data/0/int16s_values/{}".format(joint_position_choice),
                                       msg_type="fh_msgs/FhState",
                                       time_receipt=True)

        # Command data topic
        CommandTorTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(hand_choice, finger_choice),
                                      topic_field="data/0/int16s_values/{}".format(joint_choice),
                                      msg_type="fh_msgs/FhCommand",
                                      time_receipt=False)

        CommandTimeTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(hand_choice, finger_choice),
                                       topic_field="data/0/int16s_values/0",
                                       msg_type="fh_msgs/FhCommand",
                                       time_receipt=True)

        # Create configuration xml file.
        # CreatePlotConfiguration(number_of_row, number_of_columns, name_of_configuration_file)
        plots = CreatePlotConfigurations(1, 1, "base_configuration.xml")
        plots_list = plots._plots

        # Add topic to plot to the corresponding plot
        # Format - plot_list[number_of_the_plot].add_curve("name_of_the_topic_to_plot_on_x"
        # "name_of_topic_to_plot_on_y", number_of_curve_in_the_plot)
        if configuration_choice == "Raw_Encoder_Position":
            plots_list[0].set_title_and_frame_rate("{}_{}{} Position Encoder Raw Data".format(hand_choice,
                                                                                              finger_choice,
                                                                                              joint_choice), 30)
            plots_list[0].add_curve(EncoderTimeTopic, EncoderPosTopic, 0)
        elif configuration_choice == "Raw_Encoder_Torque":
            plots_list[0].set_title_and_frame_rate("{}_{}{} Torque Encoder Raw Data".format(hand_choice,
                                                                                            finger_choice,
                                                                                            joint_choice), 30)
            plots_list[0].add_curve(EncoderTimeTopic, EncoderTorTopic, 0)
        elif configuration_choice == "Position_Control":
            pass
        elif configuration_choice == "Torque_Control":
            plots_list[0].set_title_and_frame_rate("{}_{}{} CommandedTorque vs ActualTorque".format(hand_choice,
                                                                                                    finger_choice,
                                                                                                    joint_choice), 30)
            plots_list[0].add_curve(JointEffortTopic, CommandTorTopic, 0)
            pass

    def _get_joint_state_topic(self, selected_joint_name):
        for index, name in enumerate(self._joint_state_msg.name):
            if name == selected_joint_name:
                return index

    def _joint_state_callback(self, data):
        self._joint_state_msg = data
