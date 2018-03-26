#!/usr/bin/env python
#
# Copyright 2018 Shadow Robot Company Ltd.
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
        Hand H default = 1
        Finger H default = 3
        Joint H default = 3
        Add what you want with their naming
        """
        self._widget_choice['Hand'] = "1"
        self._widget_choice['Finger'] = "3"
        self._widget_choice['Joint'] = "3"
        return self._widget_choice

    def define_plot_settings(self, choices):
        """
        Add here your plot settings
        @param choices - list containing user selection from the GUI
        """
        print("Choices: ", choices)
        selected_hand = choices[0]
        selected_finger = choices[1]
        selected_joint = choices[2]
        selected_joint_position = int(selected_joint[-1:]) + int(selected_joint[-1:])
        selected_joint_torque = selected_joint_position + 1

        # Encoder data topic
        EncoderPositionDataTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(selected_hand, selected_finger),
                                               topic_field="data/0/int16s_values/{}".format(selected_joint_position),
                                               msg_type="fh_msgs/FhState")

        EncoderTorqueDataTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(selected_hand, selected_finger),
                                             topic_field="data/0/int16s_values/{}".format(selected_joint_torque),
                                             msg_type="fh_msgs/FhState")

        EncoderDataTimeTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_state".format(selected_hand, selected_finger),
                                           topic_field="data/0/int16s_values/0",
                                           msg_type="fh_msgs/FhState")

        # Command data topic
        EncoderCommandPositionTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(selected_hand, selected_finger),
                                                  topic_field="data/0/int16s_values/{}".format(selected_joint_position),
                                                  msg_type="fh_msgs/FhCommand")

        EncoderCommandTorqueTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(selected_hand, selected_finger),
                                                topic_field="data/0/int16s_values/{}".format(selected_joint_torque),
                                                msg_type="fh_msgs/FhCommand")

        EncoderCommandTimeTopic = TopicStruct(topic_name="/fh_finger/{}_{}/driver_command".format(selected_hand, selected_finger),
                                              topic_field="data/0/int16s_values/0",
                                              msg_type="fh_msgs/FhCommand")

        
        # Create configuration xml file.
        # CreatePlotConfiguration(number_of_row, number_of_columns, name_of_configuration_file)
        plots = CreatePlotConfigurations(2,2, "production_configuration.xml")

        # Get list of plots. Numbering of the plots takes first row and all the columns associated with it
        # e.g. if plot is 2 rows and 2 columns the first element of the list is plot (0,0), the second element
        # is plot(0,1), the third is (1,0) and so on. 
        plots_list = plots._plots
        print("Plot list: ", plots_list)
        # Add topic to plot to the corresponding plot
        # Format - plot_list[number_of_the_plot].add_curve("name_of_the_topic_to_plot_on_x"
        # "name_of_topic_to_plot_on_y", number_of_curve_in_the_plot)
        plots_list[0].set_title_and_frame_rate("Encoder Position Data", 30)
        plots_list[0].add_curve(EncoderPositionDataTopic, EncoderCommandTimeTopic, 0)
        plots_list[1].set_title_and_frame_rate("Encoder Torque Data", 30)
        plots_list[1].add_curve(EncoderTorqueDataTopic, EncoderDataTimeTopic, 0)

        os.system("rosrun rqt_multiplot rqt_multiplot --multiplot-config /home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/production_configuration.xml")

        return self._plot_setting_choices
