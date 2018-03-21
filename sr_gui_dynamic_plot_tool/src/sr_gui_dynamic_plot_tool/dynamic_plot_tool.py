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
from sr_utilities.hand_finder import HandFinder



class SrGuiDynamicPlotTool(Plugin):

    """
    A rosgui plugin to easily decide which hand's information to plot.
    """

    def __init__(self, context):
        super(SrGuiDynamicPlotTool, self).__init__(context)
        self.setObjectName('SrGuiDynamicPlotTool')
        print("Creating instance of Plugin")
        self._widget = QWidget()
        
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_dynamic_plot_tool'), 'uis', 'SrGuiDynamicPlotTool.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('SrGuiDynamicPlotToolUi')
        context.add_widget(self._widget)
        
        self.layout = self._widget.layout()

        self._list_scripts = os.listdir("/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/src/sr_gui_dynamic_plot_tool/scripts")

        self._widget.select_script.addItems(self._list_scripts)

        self._widget.run_button.pressed.connect(self.run_script)

    def run_script(self):
        """
        Run script selected
        """
        #add_widget("HandType")
        script_name = self._widget.select_script.currentText()
        module_name = "sr_gui_dynamic_plot_tool.scripts." + script_name[:-3]
        module = __import__(module_name, fromlist=['SrAddInterfaceEntries'])
        user_entry_class = getattr(module, 'SrAddInterfaceEntries')
        self._user_entry_class = user_entry_class()
        self._widget_choices = self._user_entry_class.populate_interface()
        self.add_widgets(self._widget_choices)

    def add_widgets(self, widget_choices):
        """
        Dynamically creates the interface for selecting plots
        """
        rospy.loginfo("Adding widget..")
        subframe = QtWidgets.QFrame()
        sublayout = QtWidgets.QHBoxLayout()
        self.plot_selection_interface = AddWidget(self._widget, widget_choices)
        sublayout.addWidget(self.plot_selection_interface)
        subframe.setLayout(sublayout)
        self.layout.addWidget(subframe)

    def create_multiplot_configuration(self):
        """
        Dynamically creates the multiplot configuration
        """
        rospy.loginfo("Creating Multiplot configuration..")


class AddWidget(QWidget):

    """
    Insert the information about the hand, finger and joint that you want to plot
    """

    def __init__(self, parent, widget_choices):
        QWidget.__init__(self, parent)
        self._widget_choices = widget_choices
        self.setWindowTitle("Plot selection interface")
        self._hand_config = HandConfig()
        plot_interface_frame = QtWidgets.QFrame()
        self.plot_interface_layout = QtWidgets.QVBoxLayout()

        self.plot_interface_layout.addWidget(plot_interface_frame)
        self.plot_interface_layout.setAlignment(Qt.AlignCenter)
        self.plot_interface_layout.setAlignment(Qt.AlignBottom)

        self.selection_button_hand = QtWidgets.QPushButton()
        self.selection_button_finger = []
        self.selection_button_joint = []

        hand_parameters, hand_prefix, hand_name = self._hand_config.get_hand_data()

        if "Hand" in widget_choices:
            self._create_hand_widget(hand_prefix, hand_name)
        if "Finger" in widget_choices:
            self._create_finger_widget(hand_parameters, hand_prefix)
        if "Joint" in widget_choices:
            self._create_joint_widget(hand_name)

        self.selection_button_hand.setCheckable(True)
        self.selection_button_hand.pressed.connect(self.hand_selected)
        
        self.setLayout(self.plot_interface_layout)
        self.show()
    
    def _create_hand_widget(self, prefix, name):
        label_name = QtWidgets.QLabel()
        label_name.setText("Found" + name)
        self.plot_interface_layout.addWidget(label_name)
        self.plot_interface_layout.addWidget(label_name)
        self.selection_button_hand.setText(prefix[:-1])
        self.plot_interface_layout.addWidget(self.selection_button_hand)
    
    def _create_finger_widget(self, hand_parameters, prefix):
        label_name = QtWidgets.QLabel()
        label_name.setText("Select Finger")
        self.plot_interface_layout.addWidget(label_name)
        for i, key in enumerate(hand_parameters[prefix[:-1]].get('fingers')):
            self.selection_button_finger = QtWidgets.QToolButton()
            self.selection_button_finger.setText(key)
            self.selection_button_finger.setObjectName(key)
            self.selection_button_finger.released.connect(self._finger_button_released)
            self.plot_interface_layout.addWidget(self.selection_button_finger)
    
    def _finger_button_released(self):
        sending_button = self.sender()
        self.finger_selected(str(sending_button.objectName()))

    def _create_joint_widget(self, hand_name):
        label_name = QtWidgets.QLabel()
        label_name.setText("Select Joint")
        self.plot_interface_layout.addWidget(label_name)
        if hand_name == "hand_h":
            number_of_joint = 3
            for joint in range(0, number_of_joint):
                self.selection_button_joint = QtWidgets.QToolButton()
                self.selection_button_joint.setText("J"+str(joint))
                self.selection_button_joint.setObjectName("J"+str(joint))
                self.selection_button_joint.released.connect(self._joint_button_released)
                self.plot_interface_layout.addWidget(self.selection_button_joint)
    
    def _joint_button_released(self):
        sending_button = self.sender()
        self.joint_selected(str(sending_button.objectName()))

    def hand_selected(self):
        print("hand selected")

    def finger_selected(self, name):
        print("finger selected", name)
    
    def joint_selected(self, name):
        print("joint selected", name)
    

class HandConfig():
    """
    Get Hand Parameters
    """
    def __init__(self):
        self.hand_finder = HandFinder()

    def get_hand_data(self):
        if self.hand_finder.hand_e_available():
            name, prefix, hand_serial = self.hand_finder.get_hand_e(number=0)
            hand_parameters = rospy.get_param("/hand")
        elif self.hand_finder.hand_h_available():
            name, prefix, hand_serial = self.hand_finder.get_hand_h(number=0)
            hand_parameters = rospy.get_param("/fh_hand")
        return hand_parameters, prefix, name

        