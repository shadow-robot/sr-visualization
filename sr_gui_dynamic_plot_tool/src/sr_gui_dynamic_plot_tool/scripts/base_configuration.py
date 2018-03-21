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


class SrAddInterfaceEntries():

    def __init__(self):
        self._widget_choice = []

    def populate_interface(self):
        """
        Add here the entries for your interface
        """
        self._widget_choice.append('Hand')
        self._widget_choice.append('Finger')
        self._widget_choice.append('Joint')
        return self._widget_choice

    def create_plots(self, choices):
        """
        Add here your plot settings
        """
        pass
        #topic_name = "/fh_finger/H" + str(choices[0]) + "J{}".format()
        #plots = create_plots(2,2)
        #plots[0].add_topic(topic1)
        #plots[0].add_topic(topic2)
        #plots.add_topic(0, topic1)
