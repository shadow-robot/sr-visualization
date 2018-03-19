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
from QtWidgets import QWidget, QMessageBox


class SrGuiDynamicPlotTool(Plugin):

    """
    A rosgui plugin to easily decide which hand's information to plot.
    """

    def __init__(self, context):
        super(SrGuiDynamicPlotTool, self).__init__(context)
        self.setObjectName('SrGuiDynamicPlotTool')

        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_dynamic_plot_tool'), 'uis', 'SrGuiDynamicPlotTool.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('SrGuiDynamicPlotToolUi')
        context.add_widget(self._widget)

