#!/usr/bin/env python

import yaml
import matplotlib
# import gc
# from mem_top import mem_top
# from pympler.tracker import SummaryTracker
# from pympler import refbrowser
# import stackimpact



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
import rviz

import time

from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64MultiArray
from sr_robot_msgs.msg import BiotacAll
from sr_gui_biotac.biotac import SrGuiBiotac

def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()

        # print '%r  %2.2f ms' % \
        #       (method.__name__, (te - ts) * 1000)
        return result
    return timed

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

        # self.create_scene_plugin_model()
        # self.create_scene_plugin_tftree()

        motor_stat_keys_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'config', 'data_visualiser_motor_stat_keys.yaml')
        parameters_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'config', 'data_visualiser_parameters.yaml')
        #gc.set_debug(gc.DEBUG_LEAK)

        with open(motor_stat_keys_file, 'r') as stream:
            try:
                self.motor_stat_keys = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        with open(parameters_file, 'r') as stream:
            try:
                data_loaded = yaml.load(stream)
                self.initialize(data_loaded)
            except yaml.YAMLError as exc:
                print(exc)

        self.setup_radio_buttons()

        self.tabWidget_motor_stats.setCurrentIndex(0)
        self.tabWidget_motor_stats.setCurrentIndex(1)
        self.tabWidget_motor_stats.setCurrentIndex(2)
        self.tabWidget_motor_stats.setCurrentIndex(3)
        self.tabWidget_motor_stats.setCurrentIndex(4)
        self.tabWidget_motor_stats.setCurrentIndex(5)
        self.tabWidget_motor_stats.setCurrentIndex(0)

        self.change_graphs(all=True, type="motor_stat", ncol=1)
        #self.include_tactile_plugin()
        #self.tracker = SummaryTracker()



        self.init_complete = True
        # TODO: refresh graphs on resize?



    def create_scene_plugin_model(self):
        package_path = rospkg.RosPack().get_path('sr_data_visualization')
        rviz_config_approach = package_path + "/uis/handescene_model.rviz"

        reader = rviz.YamlConfigReader()
        config_approach = rviz.Config()
        reader.readFile(config_approach, rviz_config_approach)
        frame_scene = rviz.VisualizationFrame()
        frame_scene.setSplashPath("")
        frame_scene.initialize()
        frame_scene.setMenuBar(None)
        frame_scene.setStatusBar(None)
        frame_scene.setHideButtonVisibility(False)
        frame_scene.load(config_approach)

        scene_layout = self._widget.findChild(QVBoxLayout, "scene_layout_model")
        scene_layout.addWidget(frame_scene)


    def create_scene_plugin_tftree(self):
        package_path = rospkg.RosPack().get_path('sr_data_visualization')
        rviz_config_approach = package_path + "/uis/handescene_tftree.rviz"

        reader = rviz.YamlConfigReader()
        config_approach = rviz.Config()
        reader.readFile(config_approach, rviz_config_approach)
        frame_scene = rviz.VisualizationFrame()
        frame_scene.setSplashPath("")
        frame_scene.initialize()
        frame_scene.setMenuBar(None)
        frame_scene.setStatusBar(None)
        frame_scene.setHideButtonVisibility(False)
        frame_scene.load(config_approach)

        scene_layout = self._widget.findChild(QVBoxLayout, "scene_layout_tf")
        scene_layout.addWidget(frame_scene)

    def include_tactile_plugin(self):
        tactile_gui = SrGuiBiotac(None, rqt_plugin=False)

        tactile_gui._widget = self._widget
        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self._widget.scrollAreaWidgetContents.update)
        self._widget.scrollAreaWidgetContents.paintEvent = tactile_gui.paintEvent

        for hand in tactile_gui._hand_parameters.mapping:
            self._widget.select_prefix.addItem(
                tactile_gui._hand_parameters.mapping[hand])
        if not tactile_gui._hand_parameters.mapping:
            rospy.logerr("No hand detected")
            # QMessageBox.warning(
            #     self._widget, "warning", "No hand is detected")
        else:
            self._widget.select_prefix.setCurrentIndex(0)

        self._widget.select_prefix.activated['QString'].connect(tactile_gui.subscribe_to_topic)

        self.timer.start(50)

        # Change background color
        p = self._widget.scrollArea.palette()
        stylesheet = """ QScrollArea>QWidget>QWidget{background: white;}"""
        self._widget.scrollArea.setStyleSheet(stylesheet)


    def tab_change_mstat(self, tab_index):
        self.hide_and_refresh(tab_index, "motor_stat")

    def tab_change(self, tab_index):
        self.hide_and_refresh(tab_index, "main")

    def hide_and_refresh(self, tab_index, tab_type):
        self.hide_tabs(tab_index, tab_type)

    def hide_tabs(self, tab_index, tab):
        #print tab_index, tab
        if tab == "main":
            if tab_index == 0:
                self.disable_graphs(["control_loops", "motor_stat", "palm_extras_accelerometer", "palm_extras_gyro", "palm_extras_adc"], disable=True)
                self.disable_graphs(["pos_vel_eff"], disable=False)
            elif tab_index == 1:
                self.disable_graphs(["pos_vel_eff", "motor_stat", "palm_extras_accelerometer", "palm_extras_gyro", "palm_extras_adc"], disable=True)
                self.disable_graphs(["control_loops"], disable=False)
            elif tab_index == 2:
                self.disable_graphs(["pos_vel_eff", "control_loops", "palm_extras_accelerometer", "palm_extras_gyro", "palm_extras_adc"], disable=True)
                self.show_specific_motor_stat_tabs()
            elif tab_index == 3:
                self.disable_graphs(["pos_vel_eff", "control_loops", "motor_stat"], disable=True)
                self.disable_graphs(["palm_extras_accelerometer", "palm_extras_gyro", "palm_extras_adc"], disable=False)
                # TODO: instead of enable all motorstat, should get motor stat tab index and enable just those

        elif tab == "motor_stat":
            #print "sub mstat tab change"
            self.show_specific_motor_stat_tabs()

    def show_specific_motor_stat_tabs(self):
        tab_index = self.tabWidget_motor_stats.currentIndex()
        #print "sub mstat tab index: ", tab_index
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
        for graph in x:
            graph.enabled = True
        x = [value for key, value in self.graph_dict_global["motor_stat"].items() if joint_group not in key]
        for graph in x:
            graph.enabled = False

    @timeit
    def disable_graphs(self, graph_type, disable):
        for element in graph_type:
            # if not disable:
            #     print "Enabling the following graphs in ", element, ": "
            # else:
            #     print "Disabling the following graphs in ", element, ": "
            for key, graph in self.graph_dict_global[element].iteritems():
                #print key
                graph.enabled = not disable

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

        self.radio_button_list = []
        j = 0
        while j < 3:
            i = 0
            while i < len(self.global_yaml["graphs"][j]["lines"]):
                tmp = self.make_all_button_functions(i, all=False, type=j)
                self.radio_button_list.append(tmp)
                i += 1
            tmp = self.make_all_button_functions(i, all=True, type=j)
            self.radio_button_list.append(tmp)
            j += 1

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

        self.radioButton_strain_gauge_left.toggled.connect(lambda: self.radio_button_list[10](self.radioButton_strain_gauge_left))
        self.radioButton_strain_gauge_right.toggled.connect(lambda: self.radio_button_list[11](self.radioButton_strain_gauge_right))
        self.radioButton_measured_pwm.toggled.connect(lambda: self.radio_button_list[12](self.radioButton_measured_pwm))
        self.radioButton_measured_current.toggled.connect(lambda: self.radio_button_list[13](self.radioButton_measured_current))
        self.radioButton_measured_voltage.toggled.connect(lambda: self.radio_button_list[14](self.radioButton_measured_voltage))
        self.radioButton_measured_effort.toggled.connect(lambda: self.radio_button_list[15](self.radioButton_measured_effort))
        self.radioButton_temperature.toggled.connect(lambda: self.radio_button_list[16](self.radioButton_temperature))
        self.radioButton_unfiltered_position.toggled.connect(lambda: self.radio_button_list[17](self.radioButton_unfiltered_position))
        self.radioButton_unfiltered_force.toggled.connect(lambda: self.radio_button_list[18](self.radioButton_unfiltered_force))
        self.radioButton_last_commanded_effort.toggled.connect(lambda: self.radio_button_list[19](self.radioButton_last_commanded_effort))
        self.radioButton_encoder_position.toggled.connect(lambda: self.radio_button_list[20](self.radioButton_encoder_position))
        self.radioButton_all_motor_stat.toggled.connect(lambda: self.radio_button_list[21](self.radioButton_all_motor_stat))

    def make_all_button_functions(self, i, all, type):
        if type == 0:
            graph_type = "pos_vel_eff"
        elif type == 1:
            graph_type = "control_loops"
        elif type == 2:
            graph_type = "motor_stat"

        if all and graph_type == "pos_vel_eff":
            number_of_columns = 3
        elif all and graph_type == "control_loops":
            number_of_columns = 3
        elif all and graph_type == "motor_stat":
            number_of_columns = 1

        if all:
            def button_function(b):
                if b.text() == "All":
                    if b.isChecked():
                        self.change_graphs(all=True, type=graph_type, ncol=number_of_columns)
        else:
            def button_function(b):
                if b.text() == self.global_yaml["graphs"][type]["lines"][i]:
                    if b.isChecked():
                        self.dump_garbage()
                        self.change_graphs(all=False, legend_name=[self.global_yaml["graphs"][type]["lines"][i]], line_number=i, type=graph_type, ncol=1)
        return button_function

    def dump_garbage(self):
        print "hey"
        #print(mem_top())  # Or just print().
        #self.tracker.print_diff()

        #
        # """
        # show us what the garbage is about
        # """
        # # Force collection
        # print "\nGARBAGE:"
        # gc.collect()
        #
        # print "\nGARBAGE OBJECTS:"
        # for x in gc.garbage:
        #     s = str(x)
        #     if len(s) > 80: s = s[:77] + '...'
        #     print type(x), "\n  ", s

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

        type = kwargs["type"]
        # TODO: speed this up. update/draw double the time taken to run this function. Only update/draw graphs we can see?
        i = 0
        while i < len(self.graph_names_global[type]):
            #self.graph_dict_global[type][self.graph_names_global[type][i]].enabled = False
            if all:
                # TODO: Store max/min somewhere? only takes approx 10uS to find_max_range, but it's a shame to call it all the time
                ymin, ymax = self.find_max_range(self.global_yaml["graphs"][index])
                self.graph_dict_global[type][self.graph_names_global[type][i]].ymin = ymin
                self.graph_dict_global[type][self.graph_names_global[type][i]].ymax = ymax
                self.graph_dict_global[type][self.graph_names_global[type][i]].plot_all = True
                self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.yaxis.set_tick_params(which='both', labelbottom=False)
                self.graph_dict_global[type][self.graph_names_global[type][i]].re_init()
                self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.legend(self.graph_dict_global[type][self.graph_names_global[type][i]].line, self.global_yaml["graphs"][index]["lines"], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5, ncol=ncols, prop={'size': 7})
            else:
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
            if graphs["ranges"][i][0] < ymin and graphs["ranges"][i][1] > ymax:
                ymin = graphs["ranges"][i][0]
                ymax = graphs["ranges"][i][1]
            i += 1
        scales = [ymin, ymax]
        return scales

    def initialize(self, data):
        self.graph_dict_global = {}
        self.control_loop_callback_dict = {}
        self.subs = []
        self.global_yaml = data
        self.graph_names_global = {}
        for graphs in data["graphs"]:
            ymin, ymax = self.find_max_range(graphs)
            self.graph_names_global[graphs["type"]] = graphs["graph_names"]

            # create_graphs
            temp_graph_dict = {}
            i = 0
            if graphs["type"] == "biotacs":
                s = 1
            while i < (len(graphs["graph_names"])):
                if graphs["type"] == "biotacs":
                    temp_graph_dict[graphs["graph_names"][i]] = CustomFigCanvas(num_lines=len(graphs["lines"]), colour=graphs["colours"], ymin=graphs["ranges"][i][0], ymax=graphs["ranges"][i][1], ranges=graphs["ranges"], graph_title=graphs["graph_names"][i], legends=graphs["lines"], legend_columns=len(graphs["lines"]), legend_font_size=7, num_ticks=4, xaxis_tick_animation=False, tail_enable=False, enabled=True)
                else:
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
            elif graphs["type"] == "palm_extras_accelerometer":
                self.subs.append(rospy.Subscriber(graphs["topic_namespace"], Float64MultiArray, self.palm_extras_cb, queue_size=1))
            elif graphs["type"] == "biotacs":
                self.subs.append(rospy.Subscriber(graphs["topic_namespace"], BiotacAll, self.biotac_all_cb, queue_size=1))

            if graphs["type"] == "palm_extras_accelerometer":
                hold = 2
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
                elif graphs["type"] == "palm_extras_accelerometer":
                    layout = graphs["graph_names"][i] + "_layout"
                elif graphs["type"] == "palm_extras_gyro":
                    layout = graphs["graph_names"][i] + "_layout"
                elif graphs["type"] == "palm_extras_adc":
                    layout = graphs["graph_names"][i] + "_layout"
                elif graphs["type"] == "biotacs":
                    layout = graphs["graph_names"][i] + "_layout"
                lay_dic[graphs["graph_names"][i]] = self._widget.findChild(QVBoxLayout, layout)
                i += 1

            # attach_graphs
            i = 0
            while i < (len(graphs["graph_names"])):
                #print graphs["graph_names"][i]
                x = lay_dic.get(graphs["graph_names"][i])
                x.addWidget(self.graph_dict_global[graphs["type"]][graphs["graph_names"][i]])
                i += 1

        #Setup palm extras graphs (as they don't need radio buttons)
        palm_extras_graphs = [value for key, value in self.graph_dict_global.items() if 'palm_extras' in key]
        i = 3
        for graph in palm_extras_graphs:
            for key, value in graph.iteritems():
                value.plot_all = True
                value.ax1.yaxis.set_tick_params(which='both', labelbottom=True)
                value.ymax = self.global_yaml["graphs"][i]["ranges"][0][1]
                value.ymin = self.global_yaml["graphs"][i]["ranges"][0][0]
                value.re_init()
                value.ax1.legend(value.line, self.global_yaml["graphs"][i]["lines"], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5, ncol=len(self.global_yaml["graphs"][i]["lines"]), prop={'size': 7})
                i += 1



    def joint_state_cb(self, value):
        if self.first_run:
            # Creates map of jointstates (so we can do joint_state.values.position[joint_state_data_map["rh_FFJ1"]]
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
                while i < len(self.graph_names_global["motor_stat"]):  # i iterate from 0 to give joint names  #j iterate from 0 to give line numbers
                    j = 0
                    while j < len(self.motor_stat_keys[1]):
                        ymin, ymax = self.find_max_range(self.global_yaml["graphs"][2])
                        graph = self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]]
                        # status_key_key = self.graph_names_global["motor_stat"][i]
                        # status_key = self.motor_stat_keys[0][string.upper(status_key_key)]
                        # data_point = data.status[status_key]
                        #
                        # lin_num_key = self.global_yaml["graphs"][2]["lines"][j]
                        # lin_num = self.motor_stat_keys[1][lin_num_key]
                        #
                        # data_values = data_point.values[lin_num]
                        # data_value = data_values.value
                        # scale = float(ymax / self.global_yaml["graphs"][2]["ranges"][j][1])
                        # if self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]].plot_all:
                        #     graph.addData(float(data_value)*scale, j)
                        # else:
                        #     graph.addData(float(data_value), j)
                        data_point = data.status[self.motor_stat_keys[0][string.upper(self.graph_names_global["motor_stat"][i])]]
                        line_number = self.motor_stat_keys[1][self.global_yaml["graphs"][2]["lines"][j]]
                        data_value = data_point.values[line_number].value
                        scale = float(ymax / self.global_yaml["graphs"][2]["ranges"][j][1])
                        if self.graph_dict_global["motor_stat"][self.graph_names_global["motor_stat"][i]].plot_all:
                            graph.addData(float(data_value)*scale, j)
                        else:
                            graph.addData(float(data_value), j)
                        j += 1
                    i += 1

    def palm_extras_cb(self, data):
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
                    graph["palm_extras_adc"].addData(data.data[9], 2)

    def biotac_all_cb(self, data):
        if self.init_complete:
            i = 0
            # graph = self.graph_dict_global["biotacs"]["TAC"]
            # point = data.tactiles[0].pdc
            # graph.addData(point, 0)
            # s = 1
            while i < len(data.tactiles):
                self.graph_dict_global["biotacs"]["PAC0"].addData(data.tactiles[i].pac0, i)
                self.graph_dict_global["biotacs"]["PAC1"].addData(data.tactiles[i].pac1, i)
                self.graph_dict_global["biotacs"]["PDC"].addData(data.tactiles[i].pdc, i)
                self.graph_dict_global["biotacs"]["TAC"].addData(data.tactiles[i].tac, i)
                self.graph_dict_global["biotacs"]["TDC"].addData(data.tactiles[i].tdc, i)
                i += 1

class CustomFigCanvas(FigureCanvas, TimedAnimation):
    @timeit
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
        self.tmp_counter = 0

    @timeit
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



    @timeit
    def new_frame_seq(self):
        return iter(range(self.n.size))

    @timeit
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
        if len(self.addedDataArray[index]) > (self.xlim*2):
            del self.addedDataArray[index][0:self.xlim]
        # self.tmp_counter += 1
        # if self.tmp_counter > 500:
        #     i = 0
        #     while i < len(self.addedDataArray):
        #         j = 0
        #         while j < len(self.addedDataArray[i]):
        #             if i > self.num_lines:
        #                 print "num_lines exceeded. i = ", i, "j = ", j
        #             if j > ((self.xlim*2) + 10):
        #                 print "j > xlim*2 + 10. j = ", j, "i = ", i
        #             j += 1
        #         i += 1
        #     self.tmp_counter = 0

        # if self.num_lines == 3:
        #     if self.tmp_counter > 131:
        #         print "joint_states"
        #         i = 0
        #         while i < self.num_lines:
        #             print len(self.addedDataArray[i])
        #             print "\n -----"
        #             i += 1
        #     self.tmp_counter += 1

            # elif self.num_lines == 5:
            #     "ctrl loop"
            #     i = 0
            #     while i < self.num_lines:
            #         print len(self.addedDataArray[i])
            #         print "\n -----"
            #         i += 1

    @timeit
    def _step(self, *args):
        if self.enabled:
            TimedAnimation._step(self, *args)

    @timeit
    def _draw_frame(self, framedata):
        # if self.enabled:
        if self.num_lines == 3:
            s = 1
        elif self.num_lines == 5:
            s = 2
        # elif self.num_lines == 11:
        #     print "sfasgdshf"
        margin = 2
        i = 0
        while i < self.num_lines:
            while len(self.addedDataArray[i]) > 0:
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
    # agent = stackimpact.start(
    #     agent_key='9e08e5540b1eb056d503dea532cc1042577da120',
    #     app_name='MyPythonApp',
    #     debug=True)
    app = QApplication(sys.argv)
    if (app.desktop().screenGeometry().width() != 2880 or app.desktop().screenGeometry().height() != 1620):
        rospy.logwarn("This program works best at a screen resolution of 2880x1620")
    planner_benchmarking_gui = SrDataVisualizer(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
