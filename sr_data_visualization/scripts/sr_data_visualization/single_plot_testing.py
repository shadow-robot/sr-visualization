#!/usr/bin/env python

import sys
import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from diagnostic_msgs.msg import DiagnosticArray

try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass
import numpy as np
import threading
import os
import time
import signal
import rospy
import rospkg
import rviz
from control_msgs.msg import JointControllerState
from sr_robot_msgs.msg import BiotacAll
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SrDataVisualizer(Plugin):
    def __init__(self, context):
        super(SrDataVisualizer, self).__init__(context)
        self.setObjectName("SrDataVisualizer")
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'uis', 'e_visualizer_test.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Dexterous Hand Data Visualizer")

        # Set white background color
        p = self._widget.palette()
        p.setColor(self._widget.backgroundRole(), Qt.white)
        self._widget.setPalette(p)

        self.tab_widget_1 = self._widget.findChild(QTabWidget, "tabWidget")
        self.tab_widget_1.currentChanged.connect(self.tab_change)


        # Change tabs background color
        p = self.tab_widget_1.palette()
        stylesheet = """ 
            QTabWidget>QWidget>QWidget{background: white;}
            """
        p.setColor(self.tab_widget_1.backgroundRole(), Qt.white)
        self.tab_widget_1.setStyleSheet(stylesheet)

        self.init_widget_children()


        self.j0_graphs_scale = 3.14159
        self.pid_output_scale = 0.013333333

        self.j0_graphs_effort_scale = self.j0_graphs_scale / 600
        self.create_graphs()
        self.create_subscribers()
        self.attach_graphs()

        self.radio_button_velocity = self._widget.findChild(QRadioButton, "radioButton_3")
        self.radio_button_all = self._widget.findChild(QRadioButton, "radioButton_4")

        self.radio_button_velocity.toggled.connect(lambda: self.joint_states_button(self.radio_button_velocity))
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
                self.change_to_vel_graphs()
                print b.text() + " is selected"
            else:
                print b.text() + " is deselected"



    def change_to_vel_graphs(self):

        # self.thj1_graph.num_lines = 1
        # self.thj1_graph.legends = ['Velocity']
        # self.thj1_graph.colour = ['blue']
        self.thj1_graph.enabled = False
        self.thj1_graph.line_to_plot = 1
        self.thj1_graph.plot_all = False
        self.thj1_graph.re_init()
        self.thj1_graph.ax1.legend(self.thj1_graph.line, ['Velocity'], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3,  mode="expand", borderaxespad=0.5)
        self.thj1_graph.enabled = True
        self.thj1_graph.update()
        self.thj1_graph.draw()

    def change_to_all_graphs(self):
        #
        # self.thj1_graph.legends = ['Position', 'Velocity', 'Effort']
        # self.thj1_graph.colour = ['red', 'blue', 'green']
        print("in ag func")
        self.thj1_graph.enabled = False
        self.thj1_graph.plot_all = True
        self.thj1_graph.re_init()
        self.thj1_graph.ax1.legend(self.thj1_graph.line, ['Position', 'Velocity', 'Effort'], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3,  mode="expand", borderaxespad=0.5, ncol=3)
        #self.thj1_graph.update()
        self.thj1_graph.enabled = True

        self.thj1_graph.draw()
        self.thj1_graph.update()

    def destroy_graph(self):
        self.thj1_layout.removeWidget(self.thj1_graph)
        self.thj1_graph.deleteLater()
        self.thj1_graph = None

    def tab_change(self, val):
        threading.Thread(target=self.delay_tab_change).start()

    def delay_tab_change(self):
        time.sleep(.300)
        self.update_graphs()
        time.sleep(.300)
        self.update_graphs()
        time.sleep(.300)
        self.update_graphs()
        time.sleep(.300)
        self.update_graphs()

    def update_graphs(self):

        self.thj1_graph.update()

    def create_graphs(self):

        j0_graphs_scale = self.j0_graphs_scale
        # All joints page graphs

        self.thj1_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale,
                                        ['Position', 'Velocity', 'Effort'])


    def create_subscribers(self):


        self.sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb, queue_size=1)


    def attach_graphs(self):

        self.thj1_layout.addWidget(self.thj1_graph)


    def thj1_pid_cb(self, value):
        self.pid_graph_thj1.addData(value.set_point, 0)
        self.pid_graph_thj1.addData(value.process_value, 1)
        self.pid_graph_thj1.addData(value.process_value_dot, 2)
        self.pid_graph_thj1.addData(value.error, 3)
        self.pid_graph_thj1.addData(value.command * self.pid_output_scale, 4)



    def joint_state_cb(self, value):

        self.thj1_graph.addData(value.position[17], 0)
        self.thj1_graph.addData(value.velocity[17], 1)
        self.thj1_graph.addData(value.effort[17] * self.j0_graphs_effort_scale, 2)


    def init_widget_children(self):

        # All joints page
        self.thj1_layout = self._widget.findChild(QVBoxLayout, "thj1_layout")


class CustomFigCanvas(FigureCanvas, TimedAnimation):
    # Inspired by: https://stackoverflow.com/questions/36665850/matplotlib-animation-inside-your-own-pyqt4-gui
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
