#!/usr/bin/env python

import yaml
import matplotlib
matplotlib.use("Qt5Agg")
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
import numpy as np
import os
import signal
import rospy
import rospkg
import string
import threading
import time

from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from diagnostic_msgs.msg import DiagnosticArray

class SrDataVisualizer(Plugin):
    def __init__(self, context):
        self.init_complete = False
        self.first_run = True
        super(SrDataVisualizer, self).__init__(context)
        self.setObjectName("SrDataVisualizer")
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'uis', 'e_visualizer_test_2.ui')
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

        self.setup_radio_buttons()

        with open("mech_stat_val_keys.yaml", 'r') as stream:
            try:
                self.motor_stat_keys = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        with open("example.yaml", 'r') as stream:
            try:
                data_loaded = yaml.load(stream)
                self.my_func(data_loaded)
            except yaml.YAMLError as exc:
                print(exc)

        # TODO: Is this still needed?
        self.tabWidget_motor_stats.setCurrentIndex(0)  # to open the first tab
        self.tabWidget_motor_stats.setCurrentIndex(1)  # to open the first tab
        self.tabWidget_motor_stats.setCurrentIndex(2)  # to open the first tab
        self.tabWidget_motor_stats.setCurrentIndex(3)  # to open the first tab
        self.tabWidget_motor_stats.setCurrentIndex(4)  # to open the first tab
        self.tabWidget_motor_stats.setCurrentIndex(5)  # to open the first tab
        self.tabWidget_motor_stats.setCurrentIndex(0)  # to open the first tab


        self.init_complete = True

# TODO: Urgent! Refactor to try and fix graph updating problems
##############################################################################################################
################################ REFACTOR EVERYTHING FROM HERE...  ###########################################
##############################################################################################################

    def tab_change_mstat(self, tab_index):
        #self.radioButton_all_motor_stat.setChecked(True)
        self.hide_and_refresh(tab_index, "motor_stat")
        # self.hide_tabs(tab_index, "motor_stat")
        # print "refreshing motor stat graphs"
        # if self.init_complete:
        #     threading.Thread(target=self.delay_tab_change(4, tab_index, "motor_stat")).start()

    def tab_change(self, tab_index):
        self.hide_and_refresh(tab_index, "main")
        # self.hide_tabs(tab_index, "main")
        # print "refreshing main graphs"
        # threading.Thread(target=self.delay_tab_change(4, tab_index, "main")).start()

    def hide_and_refresh(self, tab_index, tab_type):
        print "##########################################################"
        self.hide_tabs(tab_index, tab_type)
        self.refresh_remaining_tabs(tab_index, tab_type)

    def refresh_remaining_tabs(self, tab_index, tab_type):
        print "tab_type: ", tab_type, "tab_index", tab_index
        if tab_type == "main":
            if tab_index == 0:
                tab_group = "pos_vel_eff"
            elif tab_index == 1:
                tab_group = "control_loops"
            elif tab_index == 2:
                tab_group = "motor_stat"
        else:
            tab_group = tab_type
        print "tab_group", tab_group
        x = [value for key, value in self.graph_dict_global[tab_group].items() if value.enabled is True]

        for graph in x:
            graph.update()
        # for graph in x:
        #     graph.update()

    def hide_tabs(self, tab_index, tab):
        print tab_index, tab
        if tab == "main":
            if tab_index == 0:
                self.disable_graphs(["control_loops", "motor_stat"])
                self.enable_graphs(["pos_vel_eff"])
            elif tab_index == 1:
                self.disable_graphs(["pos_vel_eff", "motor_stat"])
                self.enable_graphs(["control_loops"])
            elif tab_index == 2:
                self.disable_graphs(["pos_vel_eff", "control_loops"])

                # TODO: instead of enable all motorstat, should get motor stat tab index and enable just those
                self.show_specific_motor_stat_tabs()

        elif tab == "motor_stat":
            print "sub mstat tab change"
            self.show_specific_motor_stat_tabs()

    def show_specific_motor_stat_tabs(self):
        tab_index = self.tabWidget_motor_stats.currentIndex()
        print "sub mstat tab index: ", tab_index
        if tab_index == 0:
            self.hide_all_but("thj")
        elif tab_index == 1:
            self.hide_all_but("ffj")
        elif tab_index == 2:
            self.hide_all_but("mfj")
        elif tab_index == 3:
            self.hide_all_but("rfj")
        elif tab_index == 4:
            self.hide_all_but("lfj")
        elif tab_index == 5:
            self.hide_all_but("wrj")

    def hide_all_but(self, joint_group):
        x = [value for key, value in self.graph_dict_global["motor_stat"].items() if joint_group in key]
        keys = [key for key, value in self.graph_dict_global["motor_stat"].items() if joint_group in key]
        print "enabling the following graphs:", keys
        for graph in x:
            graph.enabled = True
        x = [value for key, value in self.graph_dict_global["motor_stat"].items() if joint_group not in key]
        keys = [key for key, value in self.graph_dict_global["motor_stat"].items() if joint_group not in key]

        print "disabling the following graphs:", keys

        for graph in x:
            graph.enabled = False


    def enable_graphs(self, graph_type):
        for element in graph_type:
            print "enabling the following graphs in ", element, ": "
            for key, graph in self.graph_dict_global[element].iteritems():
                print key
                graph.enabled = True

    def disable_graphs(self, graph_type):
        for element in graph_type:
            print "disabling the following graphs in ", element, ": "
            for key, graph in self.graph_dict_global[element].iteritems():
                print key
                graph.enabled = False

    def delay_tab_change(self, number_of_times, tab_index, tab_group):
        i = 0
        while i < number_of_times:
            time.sleep(.300)
            self.update_graphs(tab_index, tab_group)
            i += 1
        # time.sleep(.300)
        # self.update_graphs()
        # time.sleep(.300)
        # self.update_graphs()
        # time.sleep(.300)
        # self.update_graphs()

    def update_specific(self, group):
        x = [value for key, value in self.graph_dict_global["motor_stat"].items() if group in key]
        print key, group
        s = 3
        for graph in x:
            graph.update()
            #graph.draw()


    def update_graphs(self, tab_index, tab_group):
        if tab_group == "motor_stat":
            if tab_index == 0:
                self.update_specific("thj")
            elif tab_index == 1:
                self.update_specific("ffj")
            elif tab_index == 2:
                self.update_specific("mfj")
            elif tab_index == 3:
                self.update_specific("rfj")
            elif tab_index == 4:
                self.update_specific("lfj")
            elif tab_index == 5:
                self.update_specific("wrj")
        elif tab_group == "main":
            s = 2
            for graphs in self.graph_dict_global.iteritems():
                for key, graph in graphs[1].iteritems():
                    #print key
                    graph.update()
                    graph.draw()

##############################################################################################################
##################################### TO HERE ################################################################
##############################################################################################################

    def setup_radio_buttons(self):
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
        self.radioButton_last_commanded_effort = self._widget.findChild(QRadioButton, "radioButton_last_commanded_effort")
        self.radioButton_encoder_position = self._widget.findChild(QRadioButton, "radioButton_encoder_position")

        self.radioButton_all_motor_stat.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_all_motor_stat))

        # self.radioButton_strain_gauge_left.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_strain_gauge_left))
        # self.radioButton_strain_gauge_right.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_strain_gauge_right))
        # self.radioButton_measured_pwm.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_measured_pwm))
        # self.radioButton_measured_current.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_measured_current))
        # self.radioButton_measured_voltage.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_measured_voltage))
        # self.radioButton_measured_effort.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_measured_effort))
        # self.radioButton_temperature.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_temperature))
        # self.radioButton_unfiltered_position.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_unfiltered_position))

        i = 0
        self.motor_stat_button_list = []
        #while i < len(self.motor_stat_keys[1][self.global_yaml["graphs"][2]["lines"][i]]):
        while i < 11:
            tmp = self.make_motor_stat_button_functions(i)
            self.motor_stat_button_list.append(tmp)
            i += 1

        #self.radioButton_strain_gauge_left.toggled.connect(lambda: self.motor_stat_button_list[0](self.radioButton_strain_gauge_left))
        self.radioButton_strain_gauge_left.toggled.connect(lambda: self.motor_stat_buttons(self.radioButton_strain_gauge_left))

        self.radioButton_strain_gauge_right.toggled.connect(lambda: self.motor_stat_button_list[1](self.radioButton_strain_gauge_right))
        self.radioButton_measured_pwm.toggled.connect(lambda: self.motor_stat_button_list[2](self.radioButton_measured_pwm))
        self.radioButton_measured_current.toggled.connect(lambda: self.motor_stat_button_list[3](self.radioButton_measured_current))
        self.radioButton_measured_voltage.toggled.connect(lambda: self.motor_stat_button_list[4](self.radioButton_measured_voltage))
        self.radioButton_measured_effort.toggled.connect(lambda: self.motor_stat_button_list[5](self.radioButton_measured_effort))
        self.radioButton_temperature.toggled.connect(lambda: self.motor_stat_button_list[6](self.radioButton_temperature))
        self.radioButton_unfiltered_position.toggled.connect(lambda: self.motor_stat_button_list[7](self.radioButton_unfiltered_position))
        self.radioButton_unfiltered_force.toggled.connect(lambda: self.motor_stat_button_list[8](self.radioButton_unfiltered_force))
        self.radioButton_last_commanded_effort.toggled.connect(lambda: self.motor_stat_button_list[9](self.radioButton_last_commanded_effort))
        self.radioButton_encoder_position.toggled.connect(lambda: self.motor_stat_button_list[10](self.radioButton_encoder_position))

        self.radio_button_setpoint.toggled.connect(lambda: self.control_loop_buttons(self.radio_button_setpoint))
        self.radio_button_input.toggled.connect(lambda: self.control_loop_buttons(self.radio_button_input))
        self.radio_button_dInput.toggled.connect(lambda: self.control_loop_buttons(self.radio_button_dInput))
        self.radio_button_error.toggled.connect(lambda: self.control_loop_buttons(self.radio_button_error))
        self.radio_button_output.toggled.connect(lambda: self.control_loop_buttons(self.radio_button_output))
        self.radio_button_ctrl_all.toggled.connect(lambda: self.control_loop_buttons(self.radio_button_ctrl_all))

        self.radio_button_velocity.toggled.connect(lambda: self.joint_states_button(self.radio_button_velocity))
        self.radio_button_position.toggled.connect(lambda: self.joint_states_button(self.radio_button_position))
        self.radio_button_effort.toggled.connect(lambda: self.joint_states_button(self.radio_button_effort))
        self.radio_button_all.toggled.connect(lambda: self.joint_states_button(self.radio_button_all))



    def make_motor_stat_button_functions(self, i):
        #print(i)
        def motor_stat_button(b):
            # print("btest", b.text())
            # print "line_number", i
            # print("global_yaml_line[i]: ", self.global_yaml["graphs"][2]["lines"][i])
            if b.text() == self.global_yaml["graphs"][2]["lines"][i]:
                if b.isChecked() == True:
                    self.change_graphs(all=False, legend_name=[self.global_yaml["graphs"][2]["lines"][i]], line_number=i, type="motor_stat", ncol=1)
                    #self.delay_tab_change(5)
                    #self.update_graphs()
                    print b.text() + " is selected"
                else:
                    print b.text() + " is deselected"
        return motor_stat_button

    # TODO: Refactor into dynamic function array
    def motor_stat_buttons(self, b):
        if b.text() == "All":
            if b.isChecked() == True:
                self.change_graphs(all=True, type="motor_stat", ncol=1)
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Strain Gauge Left":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['Strain Gauge Left'], line_number=0, type="motor_stat")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

    # TODO: Refactor into dynamic function array
    def control_loop_buttons(self, b):
        if b.text() == "All":
            if b.isChecked() == True:
                self.change_graphs(all=True, type="control_loops")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Setpoint":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['Setpoint'], line_number=0, type="control_loops")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Input":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['Input'], line_number=1, type="control_loops")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "dInput/dt":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['dInput/dt'], line_number=2, type="control_loops")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Error":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['Error'], line_number=3, type="control_loops")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Output":
            if b.isChecked() == True:
                # self.change_to_pos_graphs()
                self.change_graphs(all=False, legend_name=['Output'], line_number=4, type="control_loops")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

    # TODO: Refactor into dynamic function array
    def joint_states_button(self, b):
        if b.text() == "all":
            if b.isChecked() == True:
                self.change_graphs(all=True, type="pos_vel_eff")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Velocity (rad/s)":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['Velocity'], line_number=1, type="pos_vel_eff")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Effort":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['Effort'], line_number=2, type="pos_vel_eff")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "position (rad)":
            if b.isChecked() == True:
                self.change_graphs(all=False, legend_name=['Position'], line_number=0, type="pos_vel_eff")
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

    def change_graphs(self, all, **kwargs):
        if kwargs["type"] == "pos_vel_eff":
            index = 0
        elif kwargs["type"] == "control_loops":
            index = 1
        elif kwargs["type"] == "motor_stat":
            index = 2
        if 'ncol' not in kwargs:
            ncols = 3
        else:
            ncols = kwargs["ncol"]
        #print"ncols = ", ncols

        i = 0
        type = kwargs["type"]
        while i < len(self.graph_names_global[type]):
            #self.graph_dict_global[type][self.graph_names_global[type][i]].enabled = False
            if all:
                ymin, ymax = self.find_max_range(self.global_yaml["graphs"][index])
                self.graph_dict_global[type][self.graph_names_global[type][i]].ymin = ymin
                self.graph_dict_global[type][self.graph_names_global[type][i]].ymax = ymax
                self.graph_dict_global[type][self.graph_names_global[type][i]].plot_all = True
                self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.yaxis.set_tick_params(which='both', labelbottom=False)
                self.graph_dict_global[type][self.graph_names_global[type][i]].re_init()
                self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.legend(self.graph_dict_global[type][self.graph_names_global[type][i]].line, self.global_yaml["graphs"][index]["lines"], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5, ncol=ncols, prop={'size': 7})
            else:
                # print "line_name: ", self.global_yaml["graphs"][index]["ranges"][kwargs["line_number"]][1]
                # tmp = kwargs["line_number"]
                # print "line_number", tmp
                # name = kwargs["legend_name"][0]
                # print "name:", name
                self.graph_dict_global[type][self.graph_names_global[type][i]].ymin = self.global_yaml["graphs"][index]["ranges"][kwargs["line_number"]][0]
                self.graph_dict_global[type][self.graph_names_global[type][i]].ymax = self.global_yaml["graphs"][index]["ranges"][kwargs["line_number"]][1]
                self.graph_dict_global[type][self.graph_names_global[type][i]].line_to_plot = kwargs["line_number"]
                self.graph_dict_global[type][self.graph_names_global[type][i]].plot_all = False
                self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.yaxis.set_tick_params(which='both', labelbottom=True)
                self.graph_dict_global[type][self.graph_names_global[type][i]].re_init()
                self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.legend(self.graph_dict_global[type][self.graph_names_global[type][i]].line, kwargs["legend_name"], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5,  prop={'size': 7})
            #self.graph_dict_global[type][self.graph_names_global[type][i]].enabled = True
            self.graph_dict_global[type][self.graph_names_global[type][i]].update()
            self.graph_dict_global[type][self.graph_names_global[type][i]].draw()
            i += 1

    def make_control_loop_callbacks(self, graph):
        def _callback(value):
            if graph.plot_all:
                ymin, ymax = self.find_max_range(self.global_yaml["graphs"][1])
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

    def find_max_range(self, graphs):
        ymin = 0
        ymax = 0
        i = 0
        while i < len(graphs["ranges"]):
            if (graphs["ranges"][i][0] < ymin and graphs["ranges"][i][1] > ymax):
                ymin = graphs["ranges"][i][0]
                ymax = graphs["ranges"][i][1]
            i += 1
        scales = [ymin, ymax]
        return scales

    def my_func(self, data):
        self.graph_dict_global = {}
        self.control_loop_callback_dict = {}
        self.subs = []
        self.global_yaml = data
        self.graph_names_global = {}
        for graphs in data["graphs"]:
            ymin, ymax = self.find_max_range(graphs)
            self.graph_names_global[graphs["type"]] = graphs["graph_names"]
            i = 0
            # create_graphs
            temp_graph_dict = {}
            while i < (len(graphs["graph_names"])):
                temp_graph_dict[graphs["graph_names"][i]] = CustomFigCanvas(num_lines=len(graphs["lines"]), colour=graphs["colours"], ymin=ymin, ymax=ymax, ranges=graphs["ranges"], graph_title=graphs["graph_names"][i], legends=graphs["lines"], legend_columns=len(graphs["lines"]), legend_font_size=7, num_ticks=4, xaxis_tick_animation=False, tail_enable=False, enabled=True)
                i += 1
            self.graph_dict_global[graphs["type"]] = temp_graph_dict

            # create subscribers
            if graphs["type"] == "control_loops":
                i = 0
                while i < len(graphs["graph_names"]):
                    sub_namespace = graphs["topic_namespace_start"] + graphs["graph_names"][i] + graphs["topic_namespace_end"]
                    tmp_callback = self.make_control_loop_callbacks(self.graph_dict_global["control_loops"][graphs["graph_names"][i]])
                    self.subs.append(rospy.Subscriber(sub_namespace, JointControllerState, callback=tmp_callback, queue_size=1))
                    self.control_loop_callback_dict[graphs["graph_names"][i]] = tmp_callback
                    i += 1
            elif graphs["type"] == "pos_vel_eff":
                self.subs.append(rospy.Subscriber(graphs["topic_namespace"], JointState, self.joint_state_cb, queue_size=1))
            elif graphs["type"] == "motor_stat":
                self.subs.append(rospy.Subscriber(graphs["topic_namespace"], DiagnosticArray, self.diagnostic_cb, queue_size=1))


            # init_widget_children
            lay_dic = {}
            i = 0
            while i < (len(graphs["graph_names"])):
                if graphs["type"] == "pos_vel_eff":
                    layout = graphs["graph_names"][i] + "_layout"
                elif graphs["type"] == "control_loops":
                    layout = graphs["graph_names"][i] + "_layout_ctrl"
                elif graphs["type"] == "motor_stat":
                    layout = graphs["graph_names"][i] + "_layout_motor_stat"
                lay_dic[graphs["graph_names"][i]] = self._widget.findChild(QVBoxLayout, layout)
                i += 1

            # attach_graphs
            i = 0
            while i < (len(graphs["graph_names"])):
                x = lay_dic.get(graphs["graph_names"][i])
                x.addWidget(self.graph_dict_global[graphs["type"]][graphs["graph_names"][i]])
                i += 1

    def joint_state_cb(self, value):
        if self.first_run:
            #Create map of jointstates (so we can do joint_state.values.position[joint_state_data_map["rh_FFJ1"]]
            self.joint_state_data_map = {}
            i = 0
            while i < len(value.name):
                self.joint_state_data_map[value.name[i]] = i
                i += 1
            self.first_run = False
        # Only add data to graphs once they've all been created
        if self.init_complete:
            j = 0
            # for each graph
            while j < len(self.graph_names_global["pos_vel_eff"]):
                if self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]].plot_all:
                    ymin, ymax = self.find_max_range(self.global_yaml["graphs"][0])

                    # for each line
                    self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]].addData(value.position[self.joint_state_data_map['rh_' + string.upper(self.graph_names_global["pos_vel_eff"][j])]] * (ymax / self.global_yaml["graphs"][0]["ranges"][0][1]), 0)
                    self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]].addData(value.velocity[self.joint_state_data_map['rh_' + string.upper(self.graph_names_global["pos_vel_eff"][j])]] * (ymax / self.global_yaml["graphs"][0]["ranges"][1][1]), 1)
                    self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]].addData(value.effort[self.joint_state_data_map['rh_' + string.upper(self.graph_names_global["pos_vel_eff"][j])]] * (ymax / self.global_yaml["graphs"][0]["ranges"][2][1]), 2)
                    j += 1
                else:
                    # for each line
                    self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]].addData(value.position[self.joint_state_data_map['rh_' + string.upper(self.graph_names_global["pos_vel_eff"][j])]], 0)
                    self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]].addData(value.velocity[self.joint_state_data_map['rh_' + string.upper(self.graph_names_global["pos_vel_eff"][j])]], 1)
                    self.graph_dict_global["pos_vel_eff"][self.graph_names_global["pos_vel_eff"][j]].addData(value.effort[self.joint_state_data_map['rh_' + string.upper(self.graph_names_global["pos_vel_eff"][j])]], 2)
                    j += 1

    def diagnostic_cb(self, data):
        # Only add data to graphs once they've all been created
        if self.init_complete:
            if len(data.status) > 1:
                # for each joint_name / graph
                i = 0
                while i < len(self.graph_names_global["motor_stat"]):  # i iterate from 0 to give joint names            #j iterate from 0 to give line numbers
                    #print"i: ", i
                    j = 0
                    while j < len(self.motor_stat_keys[1]):
                        #print"J: ", j

                        ymin, ymax = self.find_max_range(self.global_yaml["graphs"][2])
                        graph = self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]]
                        status_key_key = self.graph_names_global["motor_stat"][i]
                        status_key = self.motor_stat_keys[0][string.upper(status_key_key)]
                        data_point = data.status[status_key]

                        lin_num_key = self.global_yaml["graphs"][2]["lines"][j]
                        lin_num = self.motor_stat_keys[1][lin_num_key]

                        data_values = data_point.values[lin_num]
                        data_value = data_values.value
                        scale = float(ymax / self.global_yaml["graphs"][2]["ranges"][j][1])
                        # print "-----------"
                        # print repr(data_value)
                        # print repr(scale)
                        # print repr(float(data_value)*scale)
                        if j == 3:
                            hold = scale
                        if self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]].plot_all:
                            graph.addData(float(data_value)*scale, j)
                        else:
                            graph.addData(float(data_value), j)
                        #
                        # ymin, ymax = self.find_max_range(self.global_yaml["graphs"][2])
                        # tmp2 = (ymax / self.global_yaml["graphs"][2]["ranges"][j][1])
                        # tmp1 = data.status[self.motor_stat_keys[0][string.upper(self.graph_names_global["motor_stat"][i])]].values[self.motor_stat_keys[1][self.global_yaml["graphs"][2]["lines"][i]]].value
                        # if self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]].plot_all:
                        #     self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]].addData(data.status[self.motor_stat_keys[0][string.upper(self.graph_names_global["motor_stat"][i])]].values[self.motor_stat_keys[1][self.global_yaml["graphs"][2]["lines"][i]]].value * (ymax / self.global_yaml["graphs"][2]["ranges"][j][1]), j)
                        # else:
                        #     self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]].addData(data.status[self.motor_stat_keys[0][string.upper(self.graph_names_global["motor_stat"][i])]].values[self.motor_stat_keys[1][self.global_yaml["graphs"][2]["lines"][i]]].value, j)

                        j += 1
                    i += 1


class CustomFigCanvas(FigureCanvas, TimedAnimation):
    # Inspired by: https://stackoverflow.com/questions/36665850/matplotlib-animation-inside-your-own-pyqt4-gui
    def __init__(self, num_lines, colour=[], ymin=-1, ymax=1, legends=[], legend_columns='none', legend_font_size=7,
                 num_ticks=4, xaxis_tick_animation=False, tail_enable=True, enabled=True, ranges=[], graph_title="oops"):
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
        if (legend_columns == 'none'):
            legend_columns = self.num_lines
        self.addedDataArray = []
        n = 0
        self.start_time = rospy.get_rostime()
        while n < self.num_lines:
            addedData = []
            self.addedDataArray.append(addedData)
            n = n + 1

        # The data
        self.xlim = 200
        self.n = np.linspace(0, self.xlim - 1, self.xlim)
        self.y = []
        n = 0
        while n < self.num_lines:
            self.y.append((self.n * 0.0) + 50)
            n = n + 1

        self.label_buffer = []

        # The window
        self.fig = Figure(figsize=(3, 3), dpi=100, facecolor=(1.0, 1.0, 1.0, 1.0))
        # self.fig.patch.set_alpha(0.0)
        self.ax1 = self.fig.add_subplot(111)
        self.x_axis = self.n
        if not (self.xaxis_tick_animation):
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
        i = 0
        while i < self.num_lines:
            self.line.append(Line2D([], [], color=self.colour[i]))
            if self.tail_enable:
                self.line_tail.append(Line2D([], [], color='red', linewidth=2))
                self.line_head.append(Line2D([], [], color='red', marker='o', markeredgecolor='r'))
                self.ax1.add_line(self.line_tail[i])
                self.ax1.add_line(self.line_head[i])
            self.ax1.add_line(self.line[i])
            self.ax1.set_xlim(0, self.xlim - 1)
            self.ax1.set_ylim(ymin, ymax)
            i = i + 1

        self.fig.subplots_adjust(bottom=0.05, top=0.8, left=0.08, right=0.98)
        self.ax1.legend(self.line, self.legends, bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3,
                        ncol=legend_columns, mode="expand", borderaxespad=0.5, prop={'size': legend_font_size})
        self.counter = 0
        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval=50, blit=not (self.xaxis_tick_animation))
        #TimedAnimation.__init__(self, self.fig, interval=50, blit=False)

    def re_init(self):
        self.line = []
        if self.tail_enable:
            self.line_head = []
            self.line_tail = []

        if self.plot_all:
            i = 0
            while i < self.num_lines:
                self.line.append(Line2D([], [], color=self.colour[i]))
                if self.tail_enable:
                    self.line_tail.append(Line2D([], [], color='red', linewidth=2))
                    self.line_head.append(Line2D([], [], color='red', marker='o', markeredgecolor='r'))
                    self.ax1.add_line(self.line_tail[i])
                    self.ax1.add_line(self.line_head[i])
                self.ax1.add_line(self.line[i])
                self.ax1.set_xlim(0, self.xlim - 1)
                self.ax1.set_ylim(self.ymin, self.ymax)
                i = i + 1
        else:
            i = 0
            while i < self.num_lines:
                self.line.append(Line2D([], [], color=self.colour[self.line_to_plot]))
                if self.tail_enable:
                    self.line_tail.append(Line2D([], [], color='red', linewidth=2))
                    self.line_head.append(Line2D([], [], color='red', marker='o', markeredgecolor='r'))
                    self.ax1.add_line(self.line_tail[i])
                    self.ax1.add_line(self.line_head[i])
                self.ax1.add_line(self.line[i])
                self.ax1.set_xlim(0, self.xlim - 1)
                self.ax1.set_ylim(self.ymin, self.ymax)
                i = i + 1

    def new_frame_seq(self):
        return iter(range(self.n.size))

    def _init_draw(self):
        i = 0
        while i < self.num_lines:
            if self.tail_enable:
                lines = [self.line[i], self.line_tail[i], self.line_head[i]]

            else:
                lines = [self.line[i]]
            for l in lines:
                l.set_data([], [])
            i = i + 1

    def addData(self, value, index):
        self.addedDataArray[index].append(value)


    def _step(self, *args):
        TimedAnimation._step(self, *args)

    def _draw_frame(self, framedata):
        if self.enabled:
            margin = 2
            i = 0
            while i < self.num_lines:
                while (len(self.addedDataArray[i]) > 0):
                    self.y[i] = np.roll(self.y[i], -1)
                    self.counter = self.counter + 1
                    self.y[i][-1] = self.addedDataArray[i][0]
                    del (self.addedDataArray[i][0])
                i = i + 1
            if self.plot_all:
                i = 0
                while i < self.num_lines:
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
                    i = i + 1
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
                i = 0
                while i < self.num_ticks:
                    x.append(int(self.xlim / self.num_ticks) * i)
                    i = i + 1
                self.ax1.set_xticks(x)
                self.ax1.set_xticklabels([int(i / int(self.xlim / self.num_ticks)) + time_from_start for i in x])

if __name__ == "__main__":
    rospy.init_node("hand_e_visualizer")
    app = QApplication(sys.argv)
    if (app.desktop().screenGeometry().width() != 2880 or app.desktop().screenGeometry().height() != 1620):
        rospy.logwarn("This program works best at a screen resolution of 2880x1620")
    planner_benchmarking_gui = SrDataVisualizer(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
