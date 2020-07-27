#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import yaml
import matplotlib
import numpy as np
import os
import signal
import rospy
import rospkg
import string
import re
import time
import atexit
matplotlib.use("Qt5Agg")  # noqa
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass
import sys
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64MultiArray
from QtGui import QIcon, QColor, QPainter, QFont
from QtWidgets import QMessageBox, QWidget
from QtCore import QRectF, QTimer
from sr_robot_msgs.msg import Biotac, BiotacAll
from sr_utilities.hand_finder import HandFinder


class SrDataVisualizer(Plugin):
    def __init__(self, context):
        self.init_complete = False
        self.first_run = True
        super(SrDataVisualizer, self).__init__(context)
        self.setObjectName("SrDataVisualizer")
        self._widget = QWidget()
        self._hand_finder = HandFinder()
        self._hand_parameters = self._hand_finder.get_hand_parameters()
        self._joint_prefix = self._hand_parameters.joint_prefix
        self._hand_g = False
        for hand, joints in self._hand_finder.hand_joints.items():
            if 'THJ3' not in joints:
                self._hand_g = True
        for key in self._hand_parameters.joint_prefix:
            self.hand_serial = key
        self._joint_prefix = self._hand_parameters.joint_prefix[self.hand_serial]
        ui_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'uis', 'hand-e_visualizer.ui')
        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)
        self._widget.setWindowTitle("Dexterous Hand Data Visualizer")

        # Set white background color
        p = self._widget.palette()
        p.setColor(self._widget.backgroundRole(), Qt.white)
        self._widget.setPalette(p)
        self.tab_widget_main = self._widget.findChild(QTabWidget, "tabWidget_main")
        self.tabWidget_motor_stats = self._widget.findChild(QTabWidget, "tabWidget_motor_stats")

        # Change tabs background color
        p = self.tab_widget_main.palette()
        stylesheet = """
            QTabWidget>QWidget>QWidget{background: white;}
            """
        p.setColor(self.tab_widget_main.backgroundRole(), Qt.white)
        self.tab_widget_main.setStyleSheet(stylesheet)

        self.tab_widget_main.currentChanged.connect(self.tab_change)
        self.tabWidget_motor_stats.currentChanged.connect(self.tab_change_mstat)

        motor_stat_keys_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'config',
                                            'data_visualiser_motor_stat_keys.yaml')
        if self._hand_g:
            parameters_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'config',
                                           'data_visualiser_parameters_rh_lite.yaml')
        elif self._joint_prefix == "rh_":
            parameters_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'config',
                                           'data_visualiser_parameters_rh.yaml')
        elif self._joint_prefix == "lh_":
            parameters_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'config',
                                           'data_visualiser_parameters_lh.yaml')
        else:
            rospy.logerr("Unknown hand detected")

        self.t0 = time.time()

        self.reset_tab_1 = self._widget.findChild(QPushButton, "reset_tab1")
        self.reset_tab_2 = self._widget.findChild(QPushButton, "reset_tab2")
        self.reset_tab_3 = self._widget.findChild(QPushButton, "reset_tab3")
        self.reset_tab_1.clicked.connect(self.reset_1)
        self.reset_tab_2.clicked.connect(self.reset_2)
        self.reset_tab_3.clicked.connect(self.reset_3)

        self.font_offset = -3

        self._widget.resizeEvent = self.on_resize_main

        self.type_dict = {
            0: "pos_vel_eff",
            1: "control_loops",
            2: "motor_stat"
        }

        self.number_of_biotacs = 5
        self.electrodes_per_biotac = 25

        with open(motor_stat_keys_file, 'r') as stream:
            try:
                self.motor_stat_keys = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        with open(parameters_file, 'r') as stream:
            try:
                data_loaded = yaml.load(stream)
                self._initialize(data_loaded)
            except yaml.YAMLError as exc:
                print(exc)

        self._setup_radio_buttons()

        self.tabWidget_motor_stats.setCurrentIndex(0)
        self.tabWidget_motor_stats.setCurrentIndex(1)
        self.tabWidget_motor_stats.setCurrentIndex(2)
        self.tabWidget_motor_stats.setCurrentIndex(3)
        self.tabWidget_motor_stats.setCurrentIndex(4)
        self.tabWidget_motor_stats.setCurrentIndex(5)
        self.tabWidget_motor_stats.setCurrentIndex(0)

        # Update legends on motor_stat graphs
        self._change_graphs(all=True, type=2, ncol=1)

        QTimer.singleShot(5000, self.reset_1)
        self.init_complete = True

    def shutdown_plugin(self):
        graph_type = [key for key, value in self.graph_names_global.items()]
        for element in graph_type:
            for key, graph in self.graph_dict_global[element].iteritems():
                graph.enabled = False
        self.init_complete = False

    def on_resize_main(self, empty):
        if (self._widget.width() * self._widget.height()) < 3500000:
            self.font_offset = -3
        else:
            self.font_offset = 1

    def reset_1(self):
        self._reset_graphs(0)
        self.radio_button_all.setChecked(True)

    def reset_2(self):
        self._reset_graphs(1)
        self.radio_button_ctrl_all.setChecked(True)

    def reset_3(self):
        self._reset_graphs(2)
        self.radioButton_all_motor_stat.setChecked(True)

    def _reset_graphs(self, tab):
        graph_type = [key for key, value in self.graph_names_global.items() if self.type_dict[tab] in key]
        for element in graph_type:
            for key, graph in self.graph_dict_global[element].iteritems():
                graph.ax1.clear()
                graph._handle_resize()
        if tab == 2:
            ncol = 1
        else:
            ncol = 3
        self._change_graphs(all=True, type=tab, ncol=ncol)

    def _include_tactile_plugin(self):
        self.tactile_gui_list = []
        self.timer = QTimer(self._widget)
        for i in range(self.number_of_biotacs):
            tactile_gui = SrGuiBiotac(None)
            tactile_gui._widget = self._widget
            tactile_gui.find_children(i)
            self.tactile_gui_list.append(tactile_gui)
            widget = self._widget.findChild(QWidget, "widget" + "_" + str(i))
            widget.resizeEvent = self.on_resize_tactile

            self.timer.timeout.connect(widget.update)
            widget.paintEvent = tactile_gui.paintEvent
            if not tactile_gui._hand_parameters.mapping:
                rospy.logerr("No hand detected")

        self.label_list = []
        self.lcd_list = []

        for j in range(self.number_of_biotacs):
            lcds = []
            labels = []
            for i in range(1, self.electrodes_per_biotac):
                if i < 10:
                    lcd = self._widget.findChild(QLCDNumber, "lcdE0" + str(i) + "_" + str(j))
                    label = self._widget.findChild(QLabel, "label_E0" + str(i) + "_" + str(j))
                else:
                    lcd = self._widget.findChild(QLCDNumber, "lcdE" + str(i) + "_" + str(j))
                    label = self._widget.findChild(QLabel, "label_E" + str(i) + "_" + str(j))
                lcds.append(lcd)
                labels.append(label)
            self.lcd_list.append(lcds)
            self.label_list.append(labels)
        self.timer.start(50)

    def on_resize_tactile(self, none):
        for tactile_widget in self.tactile_gui_list:
            tactile_widget.redraw_electrodes()

        if (self._widget.width() * self._widget.height()) < 3500000:
            scale = 6
            mheight = 18
        else:
            scale = 12
            mheight = 30

        font = QFont('Sans Serif', scale)
        font.setKerning(True)
        for j in range(self.number_of_biotacs):
            for i in range(self.electrodes_per_biotac - 1):
                self.lcd_list[j][i].setMinimumHeight(2)
                self.lcd_list[j][i].setMaximumHeight(mheight)
                self.label_list[j][i].setFont(font)

    def tab_change_mstat(self, tab_index):
        self._hide_and_refresh(tab_index, "motor_stat")

    def tab_change(self, tab_index):
        self._hide_and_refresh(tab_index, "main")

    def _hide_and_refresh(self, tab_index, tab_type):
        self._hide_tabs(tab_index, tab_type)

    def _hide_tabs(self, tab_index, tab):
        if tab == "main":
            if tab_index == 0:
                graph_type = [key for key, value in self.graph_names_global.items() if "pos_vel_eff" not in key]
                self._disable_graphs(graph_type, disable=True)
                self._disable_graphs(["pos_vel_eff"], disable=False)
            elif tab_index == 1:
                graph_type = [key for key, value in self.graph_names_global.items() if "control_loops" not in key]
                self._disable_graphs(graph_type, disable=True)
                self._disable_graphs(["control_loops"], disable=False)
            elif tab_index == 2:
                graph_type = [key for key, value in self.graph_names_global.items() if "motor_stat" not in key]
                self._disable_graphs(graph_type, disable=True)
                self._show_specific_motor_stat_tabs()
            elif tab_index == 3:
                graph_type = [key for key, value in self.graph_names_global.items() if "palm_extras" not in key]
                self._disable_graphs(graph_type, disable=True)
                graph_type = [key for key, value in self.graph_names_global.items() if "palm_extras" in key]
                self._disable_graphs(graph_type, disable=False)
            elif tab_index == 4:
                graph_type = [key for key, value in self.graph_names_global.items() if "biotacs" not in key]
                self._disable_graphs(graph_type, disable=True)
                self._disable_graphs(["biotacs"], disable=False)
            elif tab_index == 5:
                graph_type = [key for key, value in self.graph_names_global.items()]
                self._disable_graphs(graph_type, disable=True)
        elif tab == "motor_stat":
            self._show_specific_motor_stat_tabs()

    def _show_specific_motor_stat_tabs(self):
        tab_index = self.tabWidget_motor_stats.currentIndex()
        tab_index_dict = {
            0: "thj",
            1: "ffj",
            2: "mfj",
            3: "rfj",
            4: "lfj",
            5: "wrj"
        }
        self._hide_all_but(tab_index_dict[tab_index])

    def _hide_all_but(self, joint_group):
        x = [value for key, value in self.graph_dict_global["motor_stat"].items() if joint_group in key]
        for graph in x:
            graph.enabled = True
        x = [value for key, value in self.graph_dict_global["motor_stat"].items() if joint_group not in key]
        for graph in x:
            graph.enabled = False

    def _disable_graphs(self, graph_type, disable):
        for element in graph_type:
            for key, graph in self.graph_dict_global[element].iteritems():
                graph.enabled = not disable

    def _setup_radio_buttons(self):
        self.radio_button_velocity = self._widget.findChild(QRadioButton, "radioButton_velocity")
        self.radio_button_all = self._widget.findChild(QRadioButton, "radioButton_all")
        self.radio_button_position = self._widget.findChild(QRadioButton, "radioButton_position")
        self.radio_button_effort = self._widget.findChild(QRadioButton, "radioButton_effort")

        self.radio_button_setpoint = self._widget.findChild(QRadioButton, "radioButton_setpoint")
        self.radio_button_input = self._widget.findChild(QRadioButton, "radioButton_input")
        self.radio_button_dInput = self._widget.findChild(QRadioButton, "radioButton_dInput")
        self.radio_button_error = self._widget.findChild(QRadioButton, "radioButton_error")
        self.radio_button_output = self._widget.findChild(QRadioButton, "radioButton_output")
        self.radio_button_ctrl_all = self._widget.findChild(QRadioButton, "radioButton_ctrl_all")

        self.radioButton_all_motor_stat = self._widget.findChild(QRadioButton, "radioButton_all_motor_stat")
        self.radioButton_strain_gauge_left = self._widget.findChild(QRadioButton, "radioButton_strain_gauge_left")
        self.radioButton_strain_gauge_right = self._widget.findChild(QRadioButton, "radioButton_strain_gauge_right")
        self.radioButton_measured_pwm = self._widget.findChild(QRadioButton, "radioButton_measured_pwm")
        self.radioButton_measured_current = self._widget.findChild(QRadioButton, "radioButton_measured_current")
        self.radioButton_measured_voltage = self._widget.findChild(QRadioButton, "radioButton_measured_voltage")
        self.radioButton_measured_effort = self._widget.findChild(QRadioButton, "radioButton_measured_effort")
        self.radioButton_temperature = self._widget.findChild(QRadioButton, "radioButton_temperature")
        self.radioButton_unfiltered_position = self._widget.findChild(QRadioButton, "radioButton_unfiltered_position")
        self.radioButton_unfiltered_force = self._widget.findChild(QRadioButton, "radioButton_unfiltered_force")
        self.radioButton_last_commanded_effort = self._widget.findChild(QRadioButton,
                                                                        "radioButton_last_commanded_effort")
        self.radioButton_encoder_position = self._widget.findChild(QRadioButton, "radioButton_encoder_position")

        self.radio_button_list = []
        number_of_radio_button_pages = 3
        for j in range(number_of_radio_button_pages):
            for i in range(len(self.global_yaml["graphs"][j]["lines"])):
                tmp = self._make_all_button_functions(i, all=False, type=j)
                self.radio_button_list.append(tmp)
            tmp = self._make_all_button_functions(i, all=True, type=j)
            self.radio_button_list.append(tmp)

        self.radio_button_position.toggled.connect(lambda: self.radio_button_list[0](self.radio_button_position))
        self.radio_button_velocity.toggled.connect(lambda: self.radio_button_list[1](self.radio_button_velocity))
        self.radio_button_effort.toggled.connect(lambda: self.radio_button_list[2](self.radio_button_effort))
        self.radio_button_all.toggled.connect(lambda: self.radio_button_list[3](self.radio_button_all))

        self.radio_button_setpoint.toggled.connect(lambda: self.radio_button_list[4](self.radio_button_setpoint))
        self.radio_button_input.toggled.connect(lambda: self.radio_button_list[5](self.radio_button_input))
        self.radio_button_dInput.toggled.connect(lambda: self.radio_button_list[6](self.radio_button_dInput))
        self.radio_button_error.toggled.connect(lambda: self.radio_button_list[7](self.radio_button_error))
        self.radio_button_output.toggled.connect(lambda: self.radio_button_list[8](self.radio_button_output))
        self.radio_button_ctrl_all.toggled.connect(lambda: self.radio_button_list[9](self.radio_button_ctrl_all))

        self.radioButton_strain_gauge_left.toggled.connect(
            lambda: self.radio_button_list[10](self.radioButton_strain_gauge_left))
        self.radioButton_strain_gauge_right.toggled.connect(
            lambda: self.radio_button_list[11](self.radioButton_strain_gauge_right))
        self.radioButton_measured_pwm.toggled.connect(
            lambda: self.radio_button_list[12](self.radioButton_measured_pwm))
        self.radioButton_measured_current.toggled.connect(
            lambda: self.radio_button_list[13](self.radioButton_measured_current))
        self.radioButton_measured_voltage.toggled.connect(
            lambda: self.radio_button_list[14](self.radioButton_measured_voltage))
        self.radioButton_measured_effort.toggled.connect(
            lambda: self.radio_button_list[15](self.radioButton_measured_effort))
        self.radioButton_temperature.toggled.connect(
            lambda: self.radio_button_list[16](self.radioButton_temperature))
        self.radioButton_unfiltered_position.toggled.connect(
            lambda: self.radio_button_list[17](self.radioButton_unfiltered_position))
        self.radioButton_unfiltered_force.toggled.connect(
            lambda: self.radio_button_list[18](self.radioButton_unfiltered_force))
        self.radioButton_last_commanded_effort.toggled.connect(
            lambda: self.radio_button_list[19](self.radioButton_last_commanded_effort))
        self.radioButton_encoder_position.toggled.connect(
            lambda: self.radio_button_list[20](self.radioButton_encoder_position))
        self.radioButton_all_motor_stat.toggled.connect(
            lambda: self.radio_button_list[21](self.radioButton_all_motor_stat))

    def _make_all_button_functions(self, i, all, type):
        graph_type = self.type_dict[type]

        if "legend_columns" in self.global_yaml["graphs"][type]:
            number_of_columns = self.global_yaml["graphs"][type]["legend_columns"]
        elif all and graph_type == "pos_vel_eff":
            number_of_columns = 3
        elif all and graph_type == "control_loops":
            number_of_columns = 5
        elif all and graph_type == "motor_stat":
            number_of_columns = 1

        if all:
            def _button_function(b):
                if b.text() == "All":
                    if b.isChecked():
                        self._change_graphs(all=True, type=type, ncol=number_of_columns)
        else:
            legend_name = self.global_yaml["graphs"][type]["lines"][i]
            legend_name_stripped = re.sub(r"[\(\[].*?[\)\]]", "", legend_name).strip()

            def _button_function(b):
                if legend_name_stripped in b.text():
                    if b.isChecked():
                        self._change_graphs(all=False, legend_name=[legend_name],
                                            line_number=i, type=type, ncol=1)
        return _button_function

    def _change_graphs(self, all, **kwargs):
        index = kwargs["type"]
        type = self.type_dict[index]

        if 'ncol' not in kwargs:
            ncols = 3
        else:
            ncols = kwargs["ncol"]

        for i in range(len(self.graph_names_global[type])):
            graph = self.graph_dict_global[type][self.graph_names_global[type][i]]
            if all:
                ymin, ymax = self._find_max_range(self.global_yaml["graphs"][index])
                graph.ymin = ymin
                graph.ymax = ymax
                graph.plot_all = True
                graph.ax1.yaxis.set_tick_params(which='both', labelbottom=False)
                graph.re_init()
                graph.ax1.legend(graph.line, self.global_yaml["graphs"][index]["lines"],
                                 bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand",
                                 borderaxespad=0.5, ncol=ncols,
                                 prop={'size': 9 + self.font_offset})
            else:
                graph.ymin = self.global_yaml["graphs"][index]["ranges"][kwargs["line_number"]][0]
                graph.ymax = self.global_yaml["graphs"][index]["ranges"][kwargs["line_number"]][1]
                graph.line_to_plot = kwargs["line_number"]
                graph.plot_all = False
                graph.ax1.yaxis.set_tick_params(which='both', labelbottom=True, labelsize=6)
                graph.re_init()
                graph.ax1.legend(graph.line, kwargs["legend_name"], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9),
                                 framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5,
                                 prop={'size': 10 + self.font_offset})
            if graph.enabled:
                graph.update()
                graph.draw()

    def _make_control_loop_callbacks(self, graph):
        def _callback(value):
            if graph.plot_all:
                ymin, ymax = self._find_max_range(self.global_yaml["graphs"][1])
                graph.addData(value.set_point * (ymax / self.global_yaml["graphs"][1]["ranges"][0][1]), 0)
                graph.addData(value.process_value * (ymax / self.global_yaml["graphs"][1]["ranges"][1][1]), 1)
                graph.addData(value.process_value_dot * (ymax / self.global_yaml["graphs"][1]["ranges"][2][1]), 2)
                graph.addData(value.error * (ymax / self.global_yaml["graphs"][1]["ranges"][3][1]), 3)
                graph.addData(value.command * (ymax / self.global_yaml["graphs"][1]["ranges"][4][1]), 4)
            else:
                graph.addData(value.set_point, 0)
                graph.addData(value.process_value, 1)
                graph.addData(value.process_value_dot, 2)
                graph.addData(value.error, 3)
                graph.addData(value.command, 4)

        return _callback

    def _find_max_range(self, graphs):
        ymin = 0
        ymax = 0
        for i in range(len(graphs["ranges"])):
            if graphs["ranges"][i][0] <= ymin and graphs["ranges"][i][1] >= ymax:
                ymin = graphs["ranges"][i][0]
                ymax = graphs["ranges"][i][1]
        scales = [ymin, ymax]
        return scales

    def _initialize(self, data):
        self.global_yaml = data
        topic_list = rospy.get_published_topics()
        self.show_tactiles = False
        for topic in topic_list:
            for value in topic:
                if "bio" in value or "Bio" in value:
                    self._include_tactile_plugin()
                    self.show_tactiles = True
        if self.show_tactiles is False:
            for graphs in self.global_yaml["graphs"]:
                if graphs["type"] == "biotacs":
                    self.global_yaml["graphs"].remove(graphs)
            self.tab_widget_main.setTabEnabled(4, False)
            self.tab_widget_main.setTabEnabled(5, False)
            p = self.tab_widget_main.palette()
            stylesheet = """
                QTabBar::tab::disabled {width: 0; height: 0; margin: 0; padding: 1; border: none;}
                QTabWidget>QWidget>QWidget{background: white;}
                """
            p.setColor(self.tab_widget_main.backgroundRole(), Qt.white)
            self.tab_widget_main.setStyleSheet(stylesheet)

        self.graph_dict_global = {}
        self.control_loop_callback_dict = {}
        self.subs = []
        self.graph_names_global = {}
        for graphs in data["graphs"]:
            if self.show_tactiles or graphs["type"] != "biotacs":
                ymin, ymax = self._find_max_range(graphs)
                self.graph_names_global[graphs["type"]] = graphs["graph_names"]
                if "legend_columns" in graphs:
                    legend_columns = graphs["legend_columns"]
                else:
                    legend_columns = len(graphs["lines"])

                # create_graphs
                temp_graph_dict = {}
                for i in range(len(graphs["graph_names"])):
                    if graphs["type"] == "biotacs":
                        temp_graph_dict[graphs["graph_names"][i]] = CustomFigCanvas(num_lines=len(graphs["lines"]),
                                                                                    colour=graphs["colours"],
                                                                                    ymin=graphs["ranges"][i][0],
                                                                                    ymax=graphs["ranges"][i][1],
                                                                                    legends=graphs["lines"],
                                                                                    legend_columns=legend_columns,
                                                                                    legend_font_size=(
                                                                                            graphs["font_size"] +
                                                                                            self.font_offset),
                                                                                    num_ticks=4,
                                                                                    xaxis_tick_animation=False,
                                                                                    tail_enable=False, enabled=True)
                    else:
                        temp_graph_dict[graphs["graph_names"][i]] = CustomFigCanvas(num_lines=len(graphs["lines"]),
                                                                                    colour=graphs["colours"], ymin=ymin,
                                                                                    ymax=ymax, legends=graphs["lines"],
                                                                                    legend_columns=legend_columns,
                                                                                    legend_font_size=(
                                                                                            9 +
                                                                                            self.font_offset),
                                                                                    num_ticks=4,
                                                                                    xaxis_tick_animation=False,
                                                                                    tail_enable=False, enabled=True)
                self.graph_dict_global[graphs["type"]] = temp_graph_dict

                # create subscribers
                if graphs["type"] == "control_loops":
                    for i in range(len(graphs["graph_names"])):
                        sub_namespace = graphs["topic_namespace_start"] + graphs["graph_names"][i] + graphs[
                            "topic_namespace_end"]
                        tmp_callback = self._make_control_loop_callbacks(
                            self.graph_dict_global["control_loops"][graphs["graph_names"][i]])
                        self.subs.append(
                            rospy.Subscriber(sub_namespace, JointControllerState, callback=tmp_callback, queue_size=1))
                        self.control_loop_callback_dict[graphs["graph_names"][i]] = tmp_callback
                elif graphs["type"] == "pos_vel_eff":
                    self.subs.append(
                        rospy.Subscriber(graphs["topic_namespace"], JointState, self._joint_state_cb, queue_size=1))
                elif graphs["type"] == "motor_stat":
                    self.subs.append(
                        rospy.Subscriber(graphs["topic_namespace"], DiagnosticArray, self._diagnostic_cb,
                                         queue_size=1))
                elif graphs["type"] == "palm_extras_accelerometer":
                    self.subs.append(
                        rospy.Subscriber(graphs["topic_namespace"], Float64MultiArray, self._palm_extras_cb,
                                         queue_size=1))
                elif graphs["type"] == "biotacs":
                    self.subs.append(
                        rospy.Subscriber(graphs["topic_namespace"], BiotacAll, self._biotac_all_cb, queue_size=1))

                # init_widget_children
                lay_dic = {}
                for i in range(len(graphs["graph_names"])):
                    if graphs["type"] == "control_loops":
                        layout = graphs["graph_names"][i] + "_layout_ctrl"
                    elif graphs["type"] == "motor_stat":
                        layout = graphs["graph_names"][i] + "_layout_motor_stat"
                    else:
                        layout = graphs["graph_names"][i] + "_layout"
                    lay_dic[graphs["graph_names"][i]] = self._widget.findChild(QVBoxLayout, layout)
                # attach_graphs
                for i in range(len(graphs["graph_names"])):
                    x = lay_dic.get(graphs["graph_names"][i])
                    x.addWidget(self.graph_dict_global[graphs["type"]][graphs["graph_names"][i]])
        # Setup palm extras graphs (as they don't need radio buttons)
        palm_extras_graphs = [value for key, value in self.graph_dict_global.items() if 'palm_extras' in key]
        i = 3
        for graph in palm_extras_graphs:
            for key, value in graph.iteritems():
                value.plot_all = True
                value.ax1.yaxis.set_tick_params(which='both', labelbottom=True)
                value.ymax = self.global_yaml["graphs"][i]["ranges"][0][1]
                value.ymin = self.global_yaml["graphs"][i]["ranges"][0][0]
                font_size = value.ax1.legend_.prop._size
                value.re_init()
                value.ax1.legend(value.line, self.global_yaml["graphs"][i]["lines"],
                                 bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand",
                                 borderaxespad=0.5, ncol=len(self.global_yaml["graphs"][i]["lines"]),
                                 prop={'size': font_size})
                i += 1

    def _joint_state_cb(self, value):
        if self.first_run:
            # Creates map of jointstates (so we can do joint_state.values.position[joint_state_data_map["rh_FFJ1"]]
            self.joint_state_data_map = {}
            for i in range(len(value.name)):
                self.joint_state_data_map[value.name[i]] = i
            self.first_run = False
        # Only add data to graphs once they've all been created
        if self.init_complete:
            # for each graph
            for j in range(len(self.graph_names_global["pos_vel_eff"])):
                graph = self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]]
                data_index = self.joint_state_data_map[self._joint_prefix +
                                                       string.upper(self.graph_names_global["pos_vel_eff"][j])]
                if graph.plot_all:
                    ymin, ymax = self._find_max_range(self.global_yaml["graphs"][0])
                    range_array = self.global_yaml["graphs"][0]["ranges"]
                    # for each line
                    graph.addData(value.position[data_index] * (ymax / range_array[0][1]), 0)
                    graph.addData(value.velocity[data_index] * (ymax / range_array[1][1]), 1)
                    graph.addData(value.effort[data_index] * (ymax / range_array[2][1]), 2)
                else:
                    # for each line
                    graph.addData(value.position[data_index], 0)
                    graph.addData(value.velocity[data_index], 1)
                    graph.addData(value.effort[data_index], 2)

    def _diagnostic_cb(self, data):
        # Only add data to graphs once they've all been created
        if self.init_complete:
            if len(data.status) > 1:
                # for each joint_name / graph
                # i iterate from 0 to give joint names  #j iterate from 0 to give line numbers
                for i in range(len(self.graph_names_global["motor_stat"])):
                    for j in range(len(self.motor_stat_keys[1])):
                        x = self.global_yaml["graphs"][2]["lines"][j]
                        x = re.sub(r"[\(\[].*?[\)\]]", "", x).strip()
                        ymin, ymax = self._find_max_range(self.global_yaml["graphs"][2])
                        graph = self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]]
                        data_index = self.motor_stat_keys[0][string.upper(self.graph_names_global["motor_stat"][i])]
                        data_point = data.status[data_index]
                        line_number = self.motor_stat_keys[1][x]
                        try:
                            data_value = data_point.values[line_number].value
                        except IndexError as e:
                            rospy.logerr("Can't find %s. Exception: %s", data_point.name, e)
                            data_value = 0
                        scale = float(ymax / self.global_yaml["graphs"][2]["ranges"][j][1])
                        if self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]].plot_all:
                            graph.addData(float(data_value) * scale, j)
                        else:
                            graph.addData(float(data_value), j)

    def _palm_extras_cb(self, data):
        if self.init_complete:
            palm_extras_graphs = [value for key, value in self.graph_dict_global.items() if 'palm_extras' in key]
            for graph in palm_extras_graphs:
                if 'palm_extras_accelerometer' in graph:
                    graph["palm_extras_accelerometer"].addData(data.data[0], 0)
                    graph["palm_extras_accelerometer"].addData(data.data[1], 1)
                    graph["palm_extras_accelerometer"].addData(data.data[2], 2)
                if 'palm_extras_gyro' in graph:
                    graph["palm_extras_gyro"].addData(data.data[3], 0)
                    graph["palm_extras_gyro"].addData(data.data[4], 1)
                    graph["palm_extras_gyro"].addData(data.data[5], 2)
                if 'palm_extras_adc' in graph:
                    graph["palm_extras_adc"].addData(data.data[6], 0)
                    graph["palm_extras_adc"].addData(data.data[7], 1)
                    graph["palm_extras_adc"].addData(data.data[8], 2)
                    graph["palm_extras_adc"].addData(data.data[9], 3)

    def _biotac_all_cb(self, data):
        if self.init_complete:
            for i in range(len(data.tactiles)):
                self.graph_dict_global["biotacs"]["PAC0"].addData(data.tactiles[i].pac0, i)
                self.graph_dict_global["biotacs"]["PAC1"].addData(data.tactiles[i].pac1, i)
                self.graph_dict_global["biotacs"]["PDC"].addData(data.tactiles[i].pdc, i)
                self.graph_dict_global["biotacs"]["TAC"].addData(data.tactiles[i].tac, i)
                self.graph_dict_global["biotacs"]["TDC"].addData(data.tactiles[i].tdc, i)


class SrGuiBiotac(Plugin):
    _nb_electrodes_biotac = 19
    _nb_electrodes_biotac_sp = 24

    def __init__(self, context):
        super(SrGuiBiotac, self).__init__(context)
        self.setObjectName('SrGuiBiotac')
        self._hand_finder = HandFinder()
        self._hand_parameters = self._hand_finder.get_hand_parameters()
        self.load_params()

        self._widget = QWidget()

        self.latest_data = BiotacAll()
        self.pixels = self._widget.width() * self._widget.height()

        self.define_electrodes()
        self._nb_electrodes = self._nb_electrodes_biotac
        self._assign_electrodes(self._nb_electrodes)

        self._subscribe_to_topic(self.default_topic)

    def define_electrodes(self):
        self.sensing_electrodes_v1_x = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_x_locations",
                [6.45, 3.65, 3.65, 6.45, 3.65, 6.45, 0.00, 1.95, -1.95,
                 0.00, -6.45, - 3.65, -3.65, -6.45, -3.65, -6.45, 0.00,
                 0.00, 0.00])  # Physical electrode locations on the sensor
        self.sensing_electrodes_v1_y = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_y_locations",
                [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38, 6.38,
                 8.38, 7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38,
                 18.38, 22.18])

        self.excitation_electrodes_v1_x = \
            rospy.get_param(
                "sr_gui_biotac/excitation_electrodes_x_locations",
                [6.45, 3.75, -3.75, -6.45])
        self.excitation_electrodes_v1_y = \
            rospy.get_param(
                "sr_gui_biotac/excitation_electrodes_y_locations",
                [12.48, 24.48, 24.48, 12.48])

        self.sensing_electrodes_v2_x = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_x_locations",
                [5.00, 3.65, 6.45, 4.40, 2.70, 6.45, 4.40, 1.50, 4.00, 4.50,
                 -5.00, - 3.65, -6.45, -4.40, -2.70, -6.45, -4.40, -1.50, -4.00, -4.50,
                 0.00, 1.95, -1.95, 0.00])  # Physical electrode locations on the sensor
        self.sensing_electrodes_v2_y = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_y_locations",
                [4.38, 6.38, 14.78, 15.50, 18.50, 19.08, 20.00, 21.00, 23.00, 25.00,
                 4.38, 6.38, 14.78, 15.50, 18.50, 19.08, 20.00, 21.00, 23.00, 25.00,
                 7.38, 11.50, 11.50, 15.20])

        self.excitation_electrodes_v2_x = \
            rospy.get_param(
                "sr_gui_biotac/excitation_electrodes_x_locations",
                [5.30, 6.00, -5.30, -6.00])
        self.excitation_electrodes_v2_y = \
            rospy.get_param(
                "sr_gui_biotac/excitation_electrodes_y_locations",
                [9.00, 22.00, 9.00, 22.00])

    def _assign_electrodes(self, nb_electrodes):
        if nb_electrodes == self._nb_electrodes_biotac:
            self.sensing_electrodes_x = self.sensing_electrodes_v1_x
            self.sensing_electrodes_y = self.sensing_electrodes_v1_y
            self.excitation_electrodes_x = self.excitation_electrodes_v1_x
            self.excitation_electrodes_y = self.excitation_electrodes_v1_y
        elif nb_electrodes == self._nb_electrodes_biotac_sp:
            self.sensing_electrodes_x = self.sensing_electrodes_v2_x
            self.sensing_electrodes_y = self.sensing_electrodes_v2_y
            self.excitation_electrodes_x = self.excitation_electrodes_v2_x
            self.excitation_electrodes_y = self.excitation_electrodes_v2_y
        else:
            rospy.logerr("Number of electrodes %d not matching known biotac models. expected: %d or %d",
                         nb_electrodes, self._nb_electrodes_biotac, self._nb_electrodes_biotac_sp)
            return

        for n in range(len(self.sensing_electrodes_x)):
            self.sensing_electrodes_x[n] = (
                    self.sensing_electrodes_x[n] * self.factor +
                    self.x_display_offset[0])
            self.sensing_electrodes_y[n] = (
                    self.sensing_electrodes_y[n] * self.factor +
                    self.y_display_offset[0])

        for n in range(len(self.excitation_electrodes_x)):
            self.excitation_electrodes_x[n] = (
                    self.excitation_electrodes_x[n] * self.factor +
                    self.x_display_offset[0])
            self.excitation_electrodes_y[n] = (
                    self.excitation_electrodes_y[n] * self.factor +
                    self.y_display_offset[0])

    def _tactile_cb(self, msg):
        if len(msg.tactiles[0].electrodes) != self._nb_electrodes:
            self._nb_electrodes = len(msg.tactiles[0].electrodes)
            self._assign_electrodes(self._nb_electrodes)
        self.latest_data = msg

    def _get_electrode_colour_from_value(self, value):
        r = 0.0
        g = 0.0
        b = 255.0

        value = float(value)

        threshold = (0.0, 1000.0, 2000.0, 3000.0, 4095.0)

        if value <= threshold[0]:
            pass

        elif value < threshold[1]:
            r = 255
            g = 255 * ((value - threshold[0]) / (threshold[1] - threshold[0]))
            b = 0

        elif value < threshold[2]:
            r = 255 * ((threshold[2] - value) / (threshold[2] - threshold[1]))
            g = 255
            b = 0

        elif value < threshold[3]:
            r = 0
            g = 255
            b = 255 * ((value - threshold[2]) / (threshold[3] - threshold[2]))
        elif value < threshold[4]:
            r = 0
            g = 255 * ((threshold[4] - value) / (threshold[4] - threshold[3]))
            b = 255

        return QColor(r, g, b)

    def _draw_electrode(self, painter, elipse_x, elipse_y, text_x, text_y, colour, text):

        rect = QRectF(elipse_x, elipse_y, self.RECTANGLE_WIDTH,
                      self.RECTANGLE_HEIGHT)

        painter.setBrush(colour)
        painter.drawEllipse(rect)

        rect.setX(text_x)
        rect.setY(text_y)

        painter.drawText(rect, text)

    def paintEvent(self, paintEvent):
        painter = QPainter(self.widget)
        which_tactile = self.biotac_name

        painter.setFont(QFont("Arial", self.label_font_size[0]))

        if len(self.sensing_electrodes_x) == len(self.latest_data.tactiles[which_tactile].electrodes):
            for n in range(len(self.sensing_electrodes_x)):
                value = self.latest_data.tactiles[which_tactile].electrodes[n]

                eval("self._widget.lcdE%02d_%d.display(%d)" % (n + 1, self.biotac_name, value))
                colour = self._get_electrode_colour_from_value(value)

                elipse_x = self.sensing_electrodes_x[n]
                elipse_y = self.sensing_electrodes_y[n]

                if n < 9:
                    text_x = elipse_x + self.x_display_offset[1]
                    text_y = elipse_y + self.y_display_offset[1]

                else:
                    text_x = elipse_x + self.x_display_offset[2]
                    text_y = elipse_y + self.y_display_offset[2]

                self._draw_electrode(painter, elipse_x, elipse_y, text_x, text_y, colour, str(n + 1))

        painter.setFont(QFont("Arial", self.label_font_size[1]))

        for n in range(len(self.excitation_electrodes_x)):
            elipse_x = self.excitation_electrodes_x[n]
            elipse_y = self.excitation_electrodes_y[n]

            colour = QColor(127, 127, 127)

            text_x = elipse_x + self.x_display_offset[3]
            text_y = elipse_y + self.y_display_offset[3]

            self._draw_electrode(painter, elipse_x, elipse_y, text_x, text_y,
                                 colour, "X" + str(n + 1))

        self._widget.update()

    def _subscribe_to_topic(self, prefix):
        if prefix:
            rospy.Subscriber(prefix + "tactile", BiotacAll, self._tactile_cb)

    def load_params(self):

        self.RECTANGLE_WIDTH = rospy.get_param(
            "sr_gui_biotac/electrode_display_width",
            30)  # Display sizes for electrodes in pixels
        self.RECTANGLE_HEIGHT = rospy.get_param(
            "sr_gui_biotac/electrode_display_height", 30)

        self.factor = 5
        # location on the sensor in mm to display location in pixels
        self.x_display_offset = rospy.get_param(
            "sr_gui_biotac/x_display_offset", [125, 12.5, 4.5,
                                               3.5])  # Pixel offsets for
        # displaying electrodes. offset[0] is applied to each electrode.
        # 1,2 and 3 are the label offsets for displaying electrode number.
        self.y_display_offset = rospy.get_param(
            "sr_gui_biotac/y_display_offset", [-30, 4.0, 4.0, 4.0])
        self.label_font_size = rospy.get_param(
            "sr_gui_biotac/electrode_label_font_sizes", [8,
                                                         6])  # Font sizes
        # for labels on sensing + excitation electrodes

        if self._hand_parameters.mapping:
            self.default_topic = (
                    self._hand_parameters.mapping.values()[0] + '/')
        else:
            self.default_topic = ""

    def find_children(self, biotac_name):
        self.widget = self._widget.findChild(QWidget, "widget" + "_" + str(biotac_name))
        self.biotac_name = biotac_name

    def redraw_electrodes(self):
        self.pixels = self._widget.width() * self._widget.height()
        self.define_electrodes()

        if self._nb_electrodes == self._nb_electrodes_biotac:
            self.factor = 17.5
        elif self._nb_electrodes == self._nb_electrodes_biotac_sp:
            self.factor = 25.0
        else:
            rospy.logerr("Number of electrodes %d not matching known biotac models. expected: %d or %d",
                         self.nb_electrodes, self._nb_electrodes_biotac, self._nb_electrodes_biotac_sp)
            return

        if self.pixels > 3500000:
            self.factor = self.factor * 0.80
            self.RECTANGLE_WIDTH = 40
            self.RECTANGLE_HEIGHT = 40
            self.x_display_offset[0] = 170
            self.y_display_offset[0] = -30
            self.label_font_size = [10, 9]
        else:
            self.factor = self.factor * 0.55
            self.RECTANGLE_WIDTH = 25
            self.RECTANGLE_HEIGHT = 25
            self.x_display_offset[0] = 125
            self.y_display_offset[0] = -30
            self.label_font_size = [8, 8]

        self._nb_electrodes = self._nb_electrodes_biotac
        self._assign_electrodes(self._nb_electrodes)


class CustomFigCanvas(FigureCanvas, TimedAnimation):
    def __init__(self, num_lines, colour=[], ymin=-1, ymax=1, legends=[], legend_columns='none', legend_font_size=7,
                 num_ticks=4, xaxis_tick_animation=False, tail_enable=True, enabled=True):
        self.plot_all = True
        self.line_to_plot = None
        self.enabled = enabled
        self.legends = legends
        self.num_lines = num_lines
        self.num_ticks = num_ticks
        self.tail_enable = tail_enable
        self.legend_columns = legend_columns
        self.legend_font_size = legend_font_size
        self.xaxis_tick_animation = xaxis_tick_animation
        self.ymin = ymin
        self.ymax = ymax
        self.colour = colour
        if legend_columns == 'none':
            legend_columns = self.num_lines
        self.addedDataArray = []
        self.start_time = rospy.get_rostime()
        for n in range(self.num_lines):
            addedData = []
            self.addedDataArray.append(addedData)

        # The data
        self.xlim = 200
        self.n = np.linspace(0, self.xlim - 1, self.xlim)
        self.y = []
        for n in range(self.num_lines):
            self.y.append((self.n * 0.0) + 50)

        self.label_buffer = []

        # The window
        self.fig = Figure(figsize=(3, 3), dpi=100, facecolor=(1.0, 1.0, 1.0, 1.0))
        # self.fig.patch.set_alpha(0.0)
        self.ax1 = self.fig.add_subplot(111)
        self.x_axis = self.n
        if not self.xaxis_tick_animation:
            self.ax1.axes.get_xaxis().set_visible(False)

        # Shrink the font size of the x tick labels
        for tick in self.ax1.yaxis.get_major_ticks():
            tick.label.set_fontsize(7)
        self.ax1.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
        self.ax1.yaxis.set_tick_params(which='both', labelbottom=False)

        self.line = []
        if self.tail_enable:
            self.line_head = []
            self.line_tail = []
        for i in range(self.num_lines):
            self.line.append(Line2D([], [], color=self.colour[i]))
            if self.tail_enable:
                self.line_tail.append(Line2D([], [], color='red', linewidth=2))
                self.line_head.append(Line2D([], [], color='red', marker='o', markeredgecolor='r'))
                self.ax1.add_line(self.line_tail[i])
                self.ax1.add_line(self.line_head[i])
            self.ax1.add_line(self.line[i])
            self.ax1.set_xlim(0, self.xlim - 1)
            self.ax1.set_ylim(ymin, ymax)

        self.fig.subplots_adjust(bottom=0.05, top=0.8, left=0.08, right=0.98)
        self.ax1.legend(self.line, self.legends, bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3,
                        ncol=legend_columns, mode="expand", borderaxespad=0.5, prop={'size': legend_font_size})
        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval=50, blit=not self.xaxis_tick_animation)

    def re_init(self):
        self.line = []
        if self.tail_enable:
            self.line_head = []
            self.line_tail = []

        if self.plot_all:
            for i in range(self.num_lines):
                self.line.append(Line2D([], [], color=self.colour[i]))
                if self.tail_enable:
                    self.line_tail.append(Line2D([], [], color='red', linewidth=2))
                    self.line_head.append(Line2D([], [], color='red', marker='o', markeredgecolor='r'))
                    self.ax1.add_line(self.line_tail[i])
                    self.ax1.add_line(self.line_head[i])
                self.ax1.add_line(self.line[i])
                self.ax1.set_xlim(0, self.xlim - 1)
                self.ax1.set_ylim(self.ymin, self.ymax)
        else:
            for i in range(self.num_lines):
                self.line.append(Line2D([], [], color=self.colour[self.line_to_plot]))
                if self.tail_enable:
                    self.line_tail.append(Line2D([], [], color='red', linewidth=2))
                    self.line_head.append(Line2D([], [], color='red', marker='o', markeredgecolor='r'))
                    self.ax1.add_line(self.line_tail[i])
                    self.ax1.add_line(self.line_head[i])
                self.ax1.add_line(self.line[i])
                self.ax1.set_xlim(0, self.xlim - 1)
                self.ax1.set_ylim(self.ymin, self.ymax)

    def new_frame_seq(self):
        return iter(range(self.n.size))

    def _init_draw(self):
        for i in range(self.num_lines):
            if self.tail_enable:
                lines = [self.line[i], self.line_tail[i], self.line_head[i]]
            else:
                lines = [self.line[i]]
            for l in lines:
                l.set_data([], [])

    def addData(self, value, index):
        self.addedDataArray[index].append(value)
        if len(self.addedDataArray[index]) > (self.xlim * 2):
            del self.addedDataArray[index][0:self.xlim]

    def _step(self, *args):
        if self.enabled:
            TimedAnimation._step(self, *args)

    def _draw_frame(self, framedata):
        margin = 2
        for i in range(self.num_lines):
            while len(self.addedDataArray[i]) > 0:
                self.y[i] = np.roll(self.y[i], -1)
                self.y[i][-1] = self.addedDataArray[i][0]
                del (self.addedDataArray[i][0])
        if self.plot_all:
            for i in range(self.num_lines):
                self.line[i].set_data(self.n[0: self.n.size - margin], self.y[i][0: self.n.size - margin])
                if self.tail_enable:
                    self.line_tail[i].set_data(np.append(self.n[-10:-1 - margin], self.n[-1 - margin]),
                                               np.append(self.y[i][-10:-1 - margin], self.y[i][-1 - margin]))
                    self.line_head[i].set_data(self.n[-1 - margin], self.y[i][-1 - margin])
                self._drawn_artists = []
                for l in self.line:
                    self._drawn_artists.append(l)
                if self.tail_enable:
                    for l in self.line_tail:
                        self._drawn_artists.append(l)
                    for l in self.line_head:
                        self._drawn_artists.append(l)
        else:
            i = self.line_to_plot
            self.line[i].set_data(self.n[0: self.n.size - margin], self.y[i][0: self.n.size - margin])
            if self.tail_enable:
                self.line_tail[i].set_data(np.append(self.n[-10:-1 - margin], self.n[-1 - margin]),
                                           np.append(self.y[i][-10:-1 - margin], self.y[i][-1 - margin]))
                self.line_head[i].set_data(self.n[-1 - margin], self.y[i][-1 - margin])
            self._drawn_artists = []
            for l in self.line:
                self._drawn_artists.append(l)
            if self.tail_enable:
                for l in self.line_tail:
                    self._drawn_artists.append(l)
                for l in self.line_head:
                    self._drawn_artists.append(l)

        if self.xaxis_tick_animation:
            time_from_start = int((rospy.get_rostime() - self.start_time).to_sec())
            if len(self.label_buffer) > 3:
                self.label_buffer = np.roll(self.label_buffer, 1)
                self.label_buffer[2] = time_from_start
            else:
                self.label_buffer.append(time_from_start)
            x = []
            for i in range(self.num_ticks):
                x.append(int(self.xlim / self.num_ticks) * i)
            self.ax1.set_xticks(x)
            self.ax1.set_xticklabels([int(i / int(self.xlim / self.num_ticks)) + time_from_start for i in x])


if __name__ == "__main__":
    rospy.init_node("hand_e_visualizer")
    app = QApplication(sys.argv)
    data_visualiser_gui = SrDataVisualizer(None)
    data_visualiser_gui._widget.show()
    atexit.register(data_visualiser_gui.shutdown_plugin)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
