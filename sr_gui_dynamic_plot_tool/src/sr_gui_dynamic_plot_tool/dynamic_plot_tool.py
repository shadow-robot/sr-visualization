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


class SrGuiDynamicPlotToolPlugin(Plugin):

    """
    A rosgui plugin to easily decide which hand's information to plot.
    """

