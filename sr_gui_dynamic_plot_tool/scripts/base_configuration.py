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
from sr_gui_dynamic_plot_tool.dynamic_plot_tool import SrGuiDynamicPlotTool
from qt_gui.plugin import Plugin


class SrAddInterfaceEntries():

    def __init__(self):
        self._dynamic_plot_tool = SrGuiDynamicPlotTool(None)
    
    def create_interface(self):
        """
        Add here the entries for your interface
        """

        self._dynamic_plot_tool.add_widget("Hand")

        #add_widget_array("Finger", "name")

        #add_widget_topic_name("Hand", "/H?_trajectory_controller")
        #add_widget_topic_name("Next Bit", "*")

    def create_plots(self, choices):
        """
        Add here your plot settings
        """

        #topic_name = "/fh_finger/H" + str(choices[0]) + "J{}".format()

        #plots = create_plots(2,2)
        #plots[0].add_topic(topic1)
        #plots[0].add_topic(topic2)

        #plots.add_topic(0, topic1)

if __name__ == '__main__':
    rospy.init_node("dynamic_plot_configuration", anonymous=True)
    plot = SrAddInterfaceEntries()
    plot.create_interface()
    rospy.spin()

