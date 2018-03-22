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

import xml.etree.ElementTree as xmlTool



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
        self._widget_choices = self._user_entry_class.define_interface_setting()
        self.add_widgets(self._widget_choices)

    def add_widgets(self, widget_choices):
        """
        Dynamically creates the interface for selecting plots
        """
        rospy.loginfo("Adding widget..")
        subframe = QtWidgets.QFrame()
        sublayout = QtWidgets.QHBoxLayout()
        
        # Add widgets based on script selections
        self.plot_selection_interface = AddWidget(self._widget, widget_choices)
        sublayout.addWidget(self.plot_selection_interface)
        
        # Add plot button
        plot_button = QtWidgets.QPushButton()
        plot_button.setText("Plot")
        sublayout.addWidget(plot_button)

        subframe.setLayout(sublayout)
        self.layout.addWidget(subframe)

        plot_button.pressed.connect(self.get_user_choices)

    def get_user_choices(self):
        user_choices = self.plot_selection_interface._user_selections
        self._user_entry_class.define_plot_settings(user_choices)

    def create_multiplot_configuration(self, user_choices):
        """
        Dynamically creates the multiplot configuration
        """
        rospy.loginfo("Creating Multiplot configuration..")


class AddWidget(QWidget):
    """
    Create widgets to select hand, finger and joint that you want to plot
    """
    def __init__(self, parent, widget_choices):
        QWidget.__init__(self, parent)
        self._widget_choices = widget_choices
        self.setWindowTitle("Plot selection interface")
        self._hand_config = HandConfig()
        self._user_selections = []
        plot_interface_frame = QtWidgets.QFrame()
        self.plot_interface_layout = QtWidgets.QVBoxLayout()

        self.plot_interface_layout.addWidget(plot_interface_frame)
        self.plot_interface_layout.setAlignment(Qt.AlignCenter)
        self.plot_interface_layout.setAlignment(Qt.AlignBottom)

        hand_parameters, hand_prefix, hand_name = self._hand_config.get_hand_data()

        if "Hand" in widget_choices:
            self._create_hand_widget(hand_prefix, hand_name)
        if "Finger" in widget_choices:
            self._create_finger_widget(hand_parameters, hand_prefix)
        if "Joint" in widget_choices:
            self._create_joint_widget(hand_name)
        else:
            for name in widget_choices:
                self._create_generic_widget(name)
        
        self.setLayout(self.plot_interface_layout)
        self.show()
    
    def _create_hand_widget(self, prefix, name):
        """
        Create hand selection buttons
        @param prefix - string hand prefix e.g. H0, sr
        @param name - string name of the hand found
        """
        label_name = QtWidgets.QLabel()
        label_name.setText("Select " + name)
        self.plot_interface_layout.addWidget(label_name)

        selection_button_hand = QtWidgets.QToolButton()
        selection_button_hand.setCheckable(True)
        selection_button_hand.setText(prefix[:-1])
        selection_button_hand.setObjectName(prefix[:-1])

        self.plot_interface_layout.addWidget(selection_button_hand)
        selection_button_hand.released.connect(self._hand_button_released)

    def _hand_button_released(self):
        sending_button = self.sender()
        self.user_selection(str(sending_button.objectName()))
    
    def _create_finger_widget(self, hand_parameters, prefix):
        """
        Create finger selection buttons
        @param hand_parameters - dictionary that cointains hand parameters
        @param prefix - string hand prefix e.g. H0, sr
        """
        label_name = QtWidgets.QLabel()
        label_name.setText("Select Finger")
        self.plot_interface_layout.addWidget(label_name)

        for i, key in enumerate(hand_parameters[prefix[:-1]].get('fingers')):
            selection_button_finger = QtWidgets.QToolButton()
            selection_button_finger.setCheckable(True)
            selection_button_finger.setText(key)
            selection_button_finger.setObjectName(key)
            selection_button_finger.released.connect(self._finger_button_released)
            self.plot_interface_layout.addWidget(selection_button_finger)
    
    def _finger_button_released(self):
        sending_button = self.sender()
        self.user_selection(str(sending_button.objectName()))

    def _create_joint_widget(self, hand_name):
        """
        Create joint selection buttons
        @param hand_name - string name of the hand found
        """
        label_name = QtWidgets.QLabel()
        label_name.setText("Select Joint")
        self.plot_interface_layout.addWidget(label_name)

        if hand_name == "hand_h":
            number_of_joint = 3
            for joint in range(0, number_of_joint):
                selection_button_joint = QtWidgets.QToolButton()
                selection_button_joint.setCheckable(True)
                selection_button_joint.setText("J"+str(joint))
                selection_button_joint.setObjectName("J"+str(joint))
                selection_button_joint.released.connect(self._joint_button_released)
                self.plot_interface_layout.addWidget(selection_button_joint)
    
    def _create_generic_widget(self, name):
        # TODO(@anyone): update to create buttons for adding a generic widget and get options from user
        label_name = QtWidgets.QLabel()
        label_name.setText("Select"+name)
        self.plot_interface_layout.addWidget(label_name)

    def _joint_button_released(self):
        sending_button = self.sender()
        self.user_selection(str(sending_button.objectName()))

    def user_selection(self, name):
        self._user_selections.append(name)
        return self._user_selections


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


class CreatePlotConfigurations():
    """
    Dinamically create plot configuration
    """
    def __init__(self, rows, columns, configuration_name):
        self._plots = []
        self._plot_rows = []
        self._plot_columns = []
        self._base_configuration_xml = xmlTool.parse('/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/empty_configuration.xml')
        self._xml_root = self._base_configuration_xml.getroot()
        self._generate_xml(rows, columns)
        self._configuration_name = configuration_name

    def _generate_xml(self, rows, columns):
        """
        Add right number of plots in rows and columns
        """
        for child in self._xml_root.findall("table"):
            for item in child:
                if item.tag == "plots":
                    for row in range(0, rows):
                        new_row = xmlTool.SubElement(item, "row_{}".format(row))
                        for column in range(0, columns):
                            new_column = xmlTool.SubElement(new_row, "col_{}".format(column))
                            new_axes = xmlTool.SubElement(new_column, "axes")
                            new_axes_bis = xmlTool.SubElement(new_axes, "axes")
                            new_x_axis = xmlTool.SubElement(new_axes_bis, "x_axis")
                            self._set_axis(new_x_axis)
                            new_y_axis = xmlTool.SubElement(new_axes_bis, "y_axis")
                            self._set_axis(new_y_axis)
                            self._plots.append(Plot(row, column, self._configuration_name))

        self._base_configuration_xml.write("/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/{}".format(self._configuration_name))
        return self._plots

    def _set_axis(self, parent_tag):
        new_custom_title = xmlTool.SubElement(parent_tag, "custom_title")
        new_custom_title.text = "Untitled Axis"
        new_title_type = xmlTool.SubElement(parent_tag, "title_type")
        new_title_type.text = "0"
        new_title_visible = xmlTool.SubElement(parent_tag, "title_visible")
        new_title_visible.text = "true"


class Plot():
    """
    Create Plot Xml
    """
    def __init__(self, row, column, configuration_name):
        self._row = row
        self._column = column
        self._configuration_xml = xmlTool.parse('/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/{}'.format(self._configuration_name))
        self._xml_root = self._configuration_xml.getroot()

    def add_topic(self, topic_name):
        for row in self._xml_root.iter("row_{}".format(self._row)):
            print("Tags in rows: ", row.tag)
            for col in item.find("col_{}".format(self._column)):
                print("Tags in columns: ", col.tag)


            
        #add the topic to the plot depending on entrance
        # get topic and add curve
        # create the curves
        pass
