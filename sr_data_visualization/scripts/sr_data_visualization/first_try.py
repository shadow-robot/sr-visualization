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

from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState

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
        self.tab_widget_1 = self._widget.findChild(QTabWidget, "tabWidget")
        # Change tabs background color
        p = self.tab_widget_1.palette()
        stylesheet = """ 
            QTabWidget>QWidget>QWidget{background: white;}
            """
        p.setColor(self.tab_widget_1.backgroundRole(), Qt.white)
        self.tab_widget_1.setStyleSheet(stylesheet)

        self.setup_radio_buttons()


        with open("example.yaml", 'r') as stream:
            try:
                data_loaded = yaml.load(stream)
                self.my_func(data_loaded)
            except yaml.YAMLError as exc:
                print(exc)
        self.init_complete = True

    def setup_radio_buttons(self):
        self.radio_button_velocity = self._widget.findChild(QRadioButton, "radioButton_3")
        self.radio_button_all = self._widget.findChild(QRadioButton, "radioButton_4")
        self.radio_button_position = self._widget.findChild(QRadioButton, "radioButton")
        self.radio_button_effort = self._widget.findChild(QRadioButton, "radioButton_2")

        self.radio_button_velocity.toggled.connect(lambda: self.joint_states_button(self.radio_button_velocity))
        self.radio_button_position.toggled.connect(lambda: self.joint_states_button(self.radio_button_position))
        self.radio_button_effort.toggled.connect(lambda: self.joint_states_button(self.radio_button_effort))
        self.radio_button_all.toggled.connect(lambda: self.joint_states_button(self.radio_button_all))


    def joint_states_button(self, b):

        if b.text() == "all":
            if b.isChecked() == True:
                self.change_to_all_graphs()
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Velocity (rad/s)":
            if b.isChecked() == True:
                self.change_to_single_graph('Velocity', 1)
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "Effort":
            if b.isChecked() == True:
                self.change_to_single_graph('Effort', 2)
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"

        if b.text() == "position (rad)":
            if b.isChecked() == True:
                #self.change_to_pos_graphs()
                self.change_to_single_graph('Position', 0)
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"


    def change_to_single_graph(self, legend_name, line_number):
        i = 0
        temp = len(self.graph_names_joint_states)
        while i < len(self.graph_names_joint_states):
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].enabled = False
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].line_to_plot = line_number
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].plot_all = False
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ax1.yaxis.set_tick_params(which='both', labelbottom=True)
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ymin = self.global_yaml["graphs"][0]["ranges"][line_number][0]
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ymax = self.global_yaml["graphs"][0]["ranges"][line_number][1]
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].re_init()
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ax1.legend(self.graph_dict_joint_states[self.graph_names_joint_states[i]].line, [legend_name], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5, prop={'size': 7})
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].enabled = True
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].update()
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].draw()
            i += 1

    # TODO: remove redundant code in change_to_all_graphs and change_to_single_graph
    def change_to_all_graphs(self):
        ymin = 0
        ymax = 0
        i = 0
        while i < len(self.global_yaml["graphs"][0]["ranges"]):
            if (self.global_yaml["graphs"][0]["ranges"][i][0] < ymin and self.global_yaml["graphs"][0]["ranges"][i][1] > ymax):
                ymin = self.global_yaml["graphs"][0]["ranges"][i][0]
                ymax = self.global_yaml["graphs"][0]["ranges"][i][1]
            i += 1
        i = 0
        #for each graph
        while i < len(self.graph_names_joint_states):
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ymin = ymin
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ymax = ymax
            # for graph in self.graph_names_joint_states:
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].enabled = False
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].plot_all = True
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ax1.yaxis.set_tick_params(which='both', labelbottom=False)
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].re_init()
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].ax1.legend(self.graph_dict_joint_states[self.graph_names_joint_states[i]].line, ['Position', 'Velocity', 'Effort'], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3,  mode="expand", borderaxespad=0.5, ncol=3, prop={'size': 7})
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].enabled = True
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].update()
            self.graph_dict_joint_states[self.graph_names_joint_states[i]].draw()
            i += 1
    # TODO: make this work!


    def make_control_loop_callback(self, name, data):
        #print(data)
        #print(dater)
        #print(self.probably_shouldnt_do_this)
        #cb_name = self.probably_shouldnt_do_this
        def _callback(name, data):
            print("ohai")
            print("name: ", name)
        return _callback

    # def callback(self, data, name):
    #     print("ohai")
    #     # x = cb_name
    #     # print(x)
    #     #print("data:", data)
    #     print("name: ", name)

    def my_func(self, data):
        # print(data["graphs"])
        tmp = data["graphs"]
        self.graph_dict_joint_states = {}
        self.graph_dict_control_loops = {}
        self.subs = []
        self.control_loop_callbacks = []
        control_loop_cbs = []
        self.control_loop_callbacks_dict = {}
        self.global_yaml = data
        for graphs in data["graphs"]:
            if graphs["type"] == 'control_loops':
                self.graph_names_control_loops = graphs["graph_names"]
                #self.callbacks = {name: self.make_control_loop_callback for name in graphs["graph_names"]}

                i = 0
                while i < len(graphs["graph_names"]):
                    sub_namespace = graphs["topic_namespace_start"] + graphs["graph_names"][i] + graphs["topic_namespace_end"]
                    #                   self.subs.append(rospy.Subscriber(sub_namespace, JointControllerState, self.make_control_loop_callback, queue_size=1))
                    print(sub_namespace)
                    self.probably_shouldnt_do_this = graphs["graph_names"][i]
                   # self.control_loop_callbacks.append(self.callback(name=self.probably_shouldnt_do_this, dater=0))
                    temp_cb = self.make_control_loop_callback(name="ffj0", data=0)
                    #self.subs.append(rospy.Subscriber(sub_namespace, JointControllerState, control_loop_cbs.append(self.make_control_loop_callback), queue_size=1))
                    #self.subs.append(rospy.Subscriber(sub_namespace, JointControllerState, self.control_loop_callbacks.append(self.make_control_loop_callback), queue_size=1))
                    #self.subs.append(rospy.Subscriber(sub_namespace, JointControllerState, self.control_loop_callbacks.append(self.make_control_loop_callback), queue_size=1))
                    #self.subs.append(rospy.Subscriber(sub_namespace, JointControllerState, self.callbacks[graphs["graph_names"][i]], queue_size=1))
                    self.subs.append(rospy.Subscriber(sub_namespace, JointControllerState, callback=temp_cb, callback_args=graphs["graph_names"][i], queue_size=1))
                    self.control_loop_callbacks.append(temp_cb)


                    i += 1
                ymin = 0
                ymax = 0
                i = 0
                while i < len(graphs["ranges"]):
                    if (graphs["ranges"][i][0] < ymin and graphs["ranges"][i][1] > ymax):
                        ymin = graphs["ranges"][i][0]
                        ymax = graphs["ranges"][i][1]
                    i += 1
                self.graph_scales_control_loops = ymax
                i = 0

                # create_graphs
                while i < (len(graphs["graph_names"])):
                    self.graph_dict_control_loops[graphs["graph_names"][i]] = CustomFigCanvas(num_lines=len(graphs["lines"]), colour=graphs["colours"], ymin=ymin, ymax=ymax, ranges=graphs["ranges"], graph_title=graphs["graph_names"][i], legends=graphs["lines"], legend_columns=len(graphs["lines"]), legend_font_size=7, num_ticks=4, xaxis_tick_animation=False, tail_enable=True, enabled=True)
                    i += 1
                i = 0

                # init_widget_children
                lay_dic = {}
                while i < (len(graphs["graph_names"])):
                    layout = graphs["graph_names"][i] + "_layout_ctrl"
                    lay_dic[graphs["graph_names"][i]] = self._widget.findChild(QVBoxLayout, layout)
                    i += 1

                # attach_graphs
                i = 0
                while i < (len(graphs["graph_names"])):
                    x = lay_dic.get(graphs["graph_names"][i])
                    x.addWidget(self.graph_dict_control_loops[graphs["graph_names"][i]])
                    i += 1

            elif graphs["type"] == 'pos_vel_eff':
                self.graph_names_joint_states = graphs["graph_names"]
                print "------------------------"
                self.subs.append(rospy.Subscriber(graphs["topic_namespace"], JointState, self.joint_state_cb, queue_size=1))
                ymin = 0
                ymax = 0
                i = 0
                while i < len(graphs["ranges"]):
                    if (graphs["ranges"][i][0] < ymin and graphs["ranges"][i][1] > ymax):
                        ymin = graphs["ranges"][i][0]
                        ymax = graphs["ranges"][i][1]
                    i += 1
                self.graph_scales_joint_states = ymax
                i = 0

                #create_graphs
                while i < (len(graphs["graph_names"])):
                    self.graph_dict_joint_states[graphs["graph_names"][i]] = CustomFigCanvas(num_lines=len(graphs["lines"]), colour=graphs["colours"], ymin=ymin, ymax=ymax, ranges=graphs["ranges"], graph_title=graphs["graph_names"][i], legends=graphs["lines"], legend_columns=len(graphs["lines"]), legend_font_size = 7, num_ticks = 4, xaxis_tick_animation = False, tail_enable = True, enabled = True)
                    i += 1
                i = 0

                #init_widget_children
                lay_dic = {}
                while i < (len(graphs["graph_names"])):
                    layout = graphs["graph_names"][i] + "_layout"
                    lay_dic[graphs["graph_names"][i]] = self._widget.findChild(QVBoxLayout, layout)
                    i += 1

                #attach_graphs
                i = 0
                while i < (len(graphs["graph_names"])):
                    x = lay_dic.get(graphs["graph_names"][i])
                    x.addWidget(self.graph_dict_joint_states[graphs["graph_names"][i]])
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
        if self.init_complete:
            j = 0
            # for each graph
            while j < len(self.graph_names_joint_states):
                if self.graph_dict_joint_states[self.graph_names_joint_states[j]].plot_all:
                    ymax = self.graph_scales_joint_states
                    # for each line
                    self.graph_dict_joint_states[self.graph_names_joint_states[j]].addData(value.position[self.joint_state_data_map['rh_' + string.upper(self.graph_names_joint_states[j])]] * (ymax / self.global_yaml["graphs"][0]["ranges"][0][1]), 0)
                    self.graph_dict_joint_states[self.graph_names_joint_states[j]].addData(value.velocity[self.joint_state_data_map['rh_' + string.upper(self.graph_names_joint_states[j])]] * (ymax / self.global_yaml["graphs"][0]["ranges"][1][1]), 1)
                    self.graph_dict_joint_states[self.graph_names_joint_states[j]].addData(value.effort[self.joint_state_data_map['rh_' + string.upper(self.graph_names_joint_states[j])]] * (ymax / self.global_yaml["graphs"][0]["ranges"][2][1]), 2)
                    j += 1
                else:
                    # for each line
                    self.graph_dict_joint_states[self.graph_names_joint_states[j]].addData(value.position[self.joint_state_data_map['rh_' + string.upper(self.graph_names_joint_states[j])]], 0)
                    self.graph_dict_joint_states[self.graph_names_joint_states[j]].addData(value.velocity[self.joint_state_data_map['rh_' + string.upper(self.graph_names_joint_states[j])]], 1)
                    self.graph_dict_joint_states[self.graph_names_joint_states[j]].addData(value.effort[self.joint_state_data_map['rh_' + string.upper(self.graph_names_joint_states[j])]], 2)
                    j += 1



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
