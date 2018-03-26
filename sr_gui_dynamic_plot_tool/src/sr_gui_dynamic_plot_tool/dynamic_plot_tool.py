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
        Run script configuratin selected
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

        # hand_parameters, hand_prefix, hand_name = self._hand_config.get_hand_data()

        if "Hand" in widget_choices:
            hand_parameters, hand_prefix, hand_name = self._hand_config.get_hand_data()
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
        """
        Get parameters of the hands connected
        @param - number_of_hands: int that indicates the number of hands that has
        been indicated by the user in the configuration script
        """
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
        self._generate_xml(rows, columns, configuration_name)

    def _generate_xml(self, rows, columns, configuration_name):
        """
        Add right number of plots in rows and columns
        @param rows - int for the number of rows of plot that have to be added to the configuration
        @param columns - int for the number of columns in a row of plots
        @param configuration_name - string with the name of the xml file to generate
        """
        for child in self._xml_root.findall("table"):
            for item in child:
                if item.tag == "plots":
                    for row in range(0, rows):
                        new_row_tag = xmlTool.SubElement(item, "row_{}".format(row))
                        for column in range(0, columns):
                            new_column_tag = xmlTool.SubElement(new_row_tag, "col_{}".format(column))
                            new_axes_tag = xmlTool.SubElement(new_column_tag, "axes")
                            new_axes_bis_tag = xmlTool.SubElement(new_axes_tag, "axes")
                            new_x_axis_tag = xmlTool.SubElement(new_axes_bis_tag, "x_axis")
                            self._set_axis(new_x_axis_tag)
                            new_y_axis_tag = xmlTool.SubElement(new_axes_bis_tag, "y_axis")
                            self._set_axis(new_y_axis_tag)
                            curves_tag = xmlTool.SubElement(new_column_tag, "curves")
                            self._plots.append(Plot(row, column, configuration_name))

        self._base_configuration_xml.write("/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/{}".format(configuration_name))
        return self._plots

    def _set_axis(self, parent_tag):
        custom_title_tag = xmlTool.SubElement(parent_tag, "custom_title")
        custom_title_tag.text = "Untitled Axis"
        title_type_tag = xmlTool.SubElement(parent_tag, "title_type")
        title_type_tag.text = "0"
        title_visible_tag = xmlTool.SubElement(parent_tag, "title_visible")
        title_visible_tag.text = "true"


class Plot():
    """
    Create Plot Xml
    """
    def __init__(self, row, column, configuration_name):
        self._row = row
        self._column = column
        self._configuration_name = configuration_name

    def set_title_and_frame_rate(self, plot_title, frame_rate):
        """
        @param plot_title - string indicating the plot title
        @param frame_rate - int frame rate for plotting
        """
        configuration_xml = xmlTool.parse('/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/{}'.format(self._configuration_name))
        xml_root = configuration_xml.getroot()
        for row in xml_root.iter("row_{}".format(self._row)):
            for col_tag in row.iter("col_{}".format(self._column)):
                legend_tag = xmlTool.SubElement(col_tag, "legend")
                visible_legend_tag = xmlTool.SubElement(legend_tag, "visible")
                visible_legend_tag.text = "true"
                plot_rate_tag = xmlTool.SubElement(col_tag, "plot_rate")
                plot_rate_tag.text = str(frame_rate)
                plot_title_tag = xmlTool.SubElement(col_tag, "title")
                plot_title_tag.text = plot_title
        configuration_xml.write("/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/{}".format(self._configuration_name))

    def add_curve(self, x_axis_topic_name, y_axis_topic_name, curve_number):
        """
        Function to add topic to plot to the xml configuration
        @param topic_name - string that contains the topic to plot
        """
        configuration_xml = xmlTool.parse('/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/{}'.format(self._configuration_name))
        xml_root = configuration_xml.getroot()
        for row in xml_root.iter("row_{}".format(self._row)):
            for col in row.find("col_{}".format(self._column)):
                if col.tag == "curves":
                    print("Found curves: ", col.tag)
                    topic_curve_tag = xmlTool.SubElement(col, "curve_{}".format(curve_number))
                    axes_curve_tag = xmlTool.SubElement(topic_curve_tag, "axes")
                    self._add_axis_topic(axes_curve_tag, "x_axis", x_axis_topic_name)
                    self._add_axis_topic(axes_curve_tag, "y_axis", y_axis_topic_name)
                    self._add_color_settings(axes_curve_tag, "#000000")
                    self._add_data_settings(axes_curve_tag, "3")
                    self._add_style_settings(axes_curve_tag)
                    sub_queue_size_tag = xmlTool.SubElement(axes_curve_tag, "subscriber_queue_size")
                    sub_queue_size_tag.text = "100"
                    title_tag = xmlTool.SubElement(axes_curve_tag, "title")
                    title_tag.text = "test_plot"
         
        configuration_xml.write("/home/user/projects/shadow_robot/base_deps/src/sr-visualization/sr_gui_dynamic_plot_tool/xml_configurations/{}".format(self._configuration_name))

    def _add_axis_topic(self, parent_tag, name_of_axis, topic_name):
        """
        Generate x or y axis for a given curve
        @param parent_tag - parent xml tag element
        @param name_of_axis - string that contains name of axes to add, x or y
        """
        axis_tag = xmlTool.SubElement(parent_tag, name_of_axis)
        field_tag = xmlTool.SubElement(axis_tag, "field_type")
        field_tag.text = topic_name
        field_type_tag = xmlTool.SubElement(axis_tag, "field_type")
        field_type_tag.text = "1"
        scale_tag = xmlTool.SubElement(axis_tag, "scale")
        abs_max_tag = xmlTool.SubElement(scale_tag, "absolute_maximum")
        abs_max_tag.text = "1000"
        abs_min_tag = xmlTool.SubElement(scale_tag, "absolute_minimum")
        abs_min_tag.text = "1000"
        rel_max_tag = xmlTool.SubElement(scale_tag, "relative_maximum")
        rel_max_tag.text = "1000"
        rel_min_tag = xmlTool.SubElement(scale_tag, "relative_minimum")
        rel_min_tag.text = "1000"
        topic_tag = xmlTool.SubElement(axis_tag, "topic")
        topic_tag.text = topic_name
        topic_type_tag = xmlTool.SubElement(axis_tag, "topic_type")
        topic_type_tag.text = "topic_type"

    def _add_color_settings(self, parent_tag, color):
        """
        Add color settings for curve
        """
        color_tag = xmlTool.SubElement(parent_tag, "color")
        custom_color_tag = xmlTool.SubElement(color_tag, "custom_color")
        custom_color_tag.text = color
        type_color_tag = xmlTool.SubElement(color_tag, "type")
        type_color_tag.text = "0"

    def _add_data_settings(self, parent_tag, data_type):
        """
        Add data settings for curve
        """
        data_tag = xmlTool.SubElement(parent_tag, "data")
        circular_buf_capacity_tag = xmlTool.SubElement(data_tag, "circular_buffer_capacity")
        circular_buf_capacity_tag.text = "1000"
        time_frame_length_tag = xmlTool.SubElement(data_tag, "time_frame_length")
        time_frame_length_tag.text = "10"
        type_tag = xmlTool.SubElement(data_tag, "type")
        type_tag.text = data_type

    def _add_style_settings(self, parent_tag):
        """
        Add style settings for curve
        """
        style_tag = xmlTool.SubElement(parent_tag, "style")
        lines_interp_tag = xmlTool.SubElement(style_tag, "lines_interpolate")
        lines_interp_tag.text = "false"
        pen_style_tag = xmlTool.SubElement(style_tag, "pen_style")
        pen_style_tag.text = "1"
        pen_width_tag = xmlTool.SubElement(style_tag, "pen_width")
        pen_width_tag.text = "1"
        render_antialias_tag = xmlTool.SubElement(style_tag, "render_antialias")
        render_antialias_tag.text = "false"
        steps_invert_tag = xmlTool.SubElement(style_tag, "steps_invert")
        steps_invert_tag.text = "false"
        sticks_baseline_tag = xmlTool.SubElement(style_tag, "sticks_bsaeline")
        sticks_baseline_tag.text = "0"
        sticks_orientation_tag = xmlTool.SubElement(style_tag, "sticks_orientation")
        sticks_orientation_tag.text = "2"
        type_tag = xmlTool.SubElement(style_tag, "type")
        type_tag.text = "0"
