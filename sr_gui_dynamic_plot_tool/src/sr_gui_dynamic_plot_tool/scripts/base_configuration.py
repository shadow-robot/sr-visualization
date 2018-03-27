#!/usr/bin/env python
#
# Copyright 2018 Shadow Robot Company Ltd.

import os
import rospkg
import rospy
import sys
from sr_gui_dynamic_plot_tool.dynamic_plot_tool import CreatePlotConfigurations
from collections import namedtuple

TopicStruct = namedtuple('TopicStructure', "topic_name topic_field msg_type")


class SrAddInterfaceEntries():

    def __init__(self):
        self._widget_choice = {}
        self._plot_setting_choices = []

    def define_interface_setting(self):
        """
        Add here the entries for your interface.
        Format - append name of item and number of items.
        By default buttons for hand, fingers and joints will be added
        """
        self._widget_choice['Configuration'] = ["Torque", "Position"]
        return self._widget_choice

    def define_plot_settings(self, choices):
        """
        Add here your plot settings
        @param choices - list containing user selection from the GUI
        """
        selected_hand = choices[0]
        selected_finger = choices[1]
        selected_joint = choices[2]
        selected_joint_position = int(selected_joint[-1:]) + int(selected_joint[-1:])
        selected_joint_torque = selected_joint_position + 1

        # Encoder data topic
        EncoderPosTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(selected_hand, selected_finger),
                                      topic_field="data/0/int16s_values/{}".format(selected_joint_position),
                                      msg_type="fh_msgs/FhState")

        EncoderTorTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(selected_hand, selected_finger),
                                      topic_field="data/0/int16s_values/{}".format(selected_joint_torque),
                                      msg_type="fh_msgs/FhState")

        EncoderTimeTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(selected_hand, selected_finger),
                                       topic_field="data/0/int16s_values/0",
                                       msg_type="fh_msgs/FhState")

        # Command data topic
        CommandPosTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(selected_hand, selected_finger),
                                      topic_field="data/0/int16s_values/{}".format(selected_joint_position),
                                      msg_type="fh_msgs/FhCommand")

        CommandTorTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(selected_hand, selected_finger),
                                      topic_field="data/0/int16s_values/{}".format(selected_joint_torque),
                                      msg_type="fh_msgs/FhCommand")

        CommandTimeTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(selected_hand, selected_finger),
                                       topic_field="data/0/int16s_values/0",
                                       msg_type="fh_msgs/FhCommand")

        # Create configuration xml file.
        # CreatePlotConfiguration(number_of_row, number_of_columns, name_of_configuration_file)
        plots = CreatePlotConfigurations(1, 2, "base_configuration.xml")
        plots_list = plots._plots

        # Add topic to plot to the corresponding plot
        # Format - plot_list[number_of_the_plot].add_curve("name_of_the_topic_to_plot_on_x"
        # "name_of_topic_to_plot_on_y", number_of_curve_in_the_plot)
        plots_list[0].set_title_and_frame_rate("{}_{} Position Encoder Raw Data".format(selected_hand, selected_finger), 30)
        plots_list[0].add_curve(EncoderPosTopic, EncoderTimeTopic, 0)
        plots_list[1].set_title_and_frame_rate("{}_{} Torque Encoder Raw Data".format(selected_hand, selected_finger), 30)
        plots_list[1].add_curve(EncoderTorTopic, EncoderTimeTopic, 0)

        xml_config = os.path.expanduser("~/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations")
        os.system("rosrun rqt_multiplot rqt_multiplot --multiplot-config {}/production_configuration.xml".format(xml_config))

        return self._plot_setting_choices
