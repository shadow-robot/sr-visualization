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
import QtWidgets
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
        
        self.layout = self._widget.layout()

        self._list_scripts = os.listdir("/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/scripts")
        self._widget.select_script.addItems(self._list_scripts)

        self._widget.run_button.pressed.connect(self.run_script)

    def run_script(self):
        """
        Run script selected
        """
        script_name = self._widget.select_script.currentText()
        os.system("python /home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/scripts/{}".format(script_name))

    def add_widget(self, name):
        """
        Dynamically creates the interface for selecting plots
        """
        rospy.loginfo("Adding widget..")
        subframe = QtWidgets.QFrame()
        sublayout = QtWidgets.QHBoxLayout()
        self.plot_selection_interface = PlotSelectionInterface(self._widget, name)
        sublayout.addWidget(self.plot_selection_interface)
        subframe.setLayout(sublayout)
        self.layout.addWidget(subframe)

    def create_multiplot_configuration(self):
        """
        Dynamically creates the multiplot configuration
        """
        rospy.loginfo("Creating Multiplot configuration..")


class PlotSelectionInterface(QWidget):

    """
    Insert the information about the hand, finger and joint that you want to plot
    """

    def __init__(self, name, parent):
        QWidget.__init__(self, name, parent=parent)
        self.setWindowTitle("Plot selection interface")
        plot_interface_frame = QtWidgets.QFrame()
        plot_interface_layout = QtWidgets.QFormLayout()

        plot_interface_layout.addWidget(plot_interface_frame)
        plot_interface_layout.setAlignment(Qt.AlignCenter)
        plot_interface_layout.setAlignment(Qt.AlignBottom)

        # define and add hand to widget
        label_name = QtWidgets.QLabel()
        label_name.setText("select " + name)
        self.hand_entry_box = QtWidgets.QLineEdit()
        self.hand_entry_box.setFixedWidth(30)
        plot_interface_layout.addRow(hand_label_name, self.hand_entry_box)

        # define and add finger box to widget
        #finger_label_name = QtWidgets.QLabel()
        #finger_label_name.setText("Finger name: ")
        #self.finger_entry_box = QtWidgets.QLineEdit()
        #self.finger_entry_box.setFixedWidth(30)
        #plot_interface_layout.addRow(finger_label_name, self.finger_entry_box)

        # define and add joint box to widget
        #joint_label_name = QtWidgets.QLabel()
        #joint_label_name.setText("Joint name: ")
        #self.joint_entry_box = QtWidgets.QLineEdit()
        #self.joint_entry_box.setFixedWidth(30)
        #plot_interface_layout.addRow(joint_label_name, self.joint_entry_box)

        # Create and show layout
        self.setLayout(plot_interface_layout)
        self.show()

        # Get entries
        self.hand_entry_box.returnPressed.connect(self.hand_name_changed)
        self.finger_entry_box.returnPressed.connect(self.finger_name_changed)
        self.joint_entry_box.returnPressed.connect(self.joint_name_changed)
       
    def hand_name_changed(self):
        hand_name = self.hand_entry_box.text()
        print("Hand name ", hand_name)

    def finger_name_changed(self, name):
        finger_name = self.finger_entry_box.text()
        print("Finger name ", finger_name)

    def joint_name_changed(self, name):
        joint_name = self.joint_entry_box.text()
        print("Joint name ", joint_name)