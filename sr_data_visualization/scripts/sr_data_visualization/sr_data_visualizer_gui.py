#!/usr/bin/env python

import sys
import matplotlib
matplotlib.use("Qt5Agg")
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import numpy as np
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D

import signal
import rospy
from control_msgs.msg import JointControllerState
from sr_robot_msgs.msg import MechanismStatistics
from sensor_msgs.msg import JointState
import os
import rospkg
import rviz
import subprocess



class SrDataVisualizer(Plugin):
    def __init__(self, context):
        super(SrDataVisualizer, self).__init__(context)
        self.setObjectName("SrDataVisualizer")
        self._widget = QWidget()
        self.create_menu_bar()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_data_visualization'), 'uis', 'hand-e_visualizer.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Moveit Planner Benchmarks")
        self.init_widget_children()
        self.create_scene_plugin()

        j0_graphs_scale = 3.14159
        #j0_graphs_scale = 600
        self.j0_graphs_effort_scale = j0_graphs_scale/600
        #self.init_plots()
        self.graph_one = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'yellow'], -4, 4, ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.graph_two = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'yellow'], -300, 300, ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.graph_three = CustomFigCanvas(3, ['red', 'blue', 'green'], -100, 100, ['Position', 'Velocity', 'Effort'])
        self.j0_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j1_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j2_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j3_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j4_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j5_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j6_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j7_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j8_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j9_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j10_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j11_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j12_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j13_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j14_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j15_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j16_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j17_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j18_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j19_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j20_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j21_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j22_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])
        self.j23_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -j0_graphs_scale, j0_graphs_scale, ['Position', 'Velocity', 'Effort'])

        #self.graph_two = CustomFigCanvas(1, ['blue'], -3.5, 4.2, ['one'])
        #self.create_graph(self.plan_time_layout)

        self.sub = rospy.Subscriber('sh_rh_ffj0_position_controller/state', JointControllerState,
                                    self.p_val_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('sh_rh_ffj0_position_controller/state', JointControllerState,
                                    self.p_val_dot_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('mechanism_statistics', MechanismStatistics, self.mech_stat_cb, queue_size=1)


        self.sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb, queue_size=1)

        #self.graph_one.setParent(self._widget)
        #self.graph_two = CustomFigCanvas()
        #self.graph_three = CustomFigCanvas()

        self.plan_time_layout.addWidget(self.graph_one)
        self.finger_position_layout.addWidget(self.graph_two)
        self.pos_vel_eff_layout.addWidget(self.graph_three)

        self.j0_layout.addWidget(self.j0_graph)
        self.j1_layout.addWidget(self.j1_graph)
        self.j2_layout.addWidget(self.j2_graph)
        self.j3_layout.addWidget(self.j3_graph)
        self.j4_layout.addWidget(self.j4_graph)
        self.j5_layout.addWidget(self.j5_graph)
        self.j6_layout.addWidget(self.j6_graph)
        self.j7_layout.addWidget(self.j7_graph)
        self.j8_layout.addWidget(self.j8_graph)

        self.j9_layout.addWidget(self.j9_graph)
        self.j10_layout.addWidget(self.j10_graph)
        self.j11_layout.addWidget(self.j11_graph)
        self.j12_layout.addWidget(self.j12_graph)
        self.j13_layout.addWidget(self.j13_graph)
        self.j14_layout.addWidget(self.j14_graph)
        self.j15_layout.addWidget(self.j15_graph)
        self.j16_layout.addWidget(self.j16_graph)
        self.j17_layout.addWidget(self.j17_graph)

        self.j18_layout.addWidget(self.j18_graph)
        self.j19_layout.addWidget(self.j19_graph)
        self.j20_layout.addWidget(self.j20_graph)
        self.j21_layout.addWidget(self.j21_graph)
        self.j22_layout.addWidget(self.j22_graph)
        self.j23_layout.addWidget(self.j23_graph)

    def joint_state_cb(self, value):
        self.j0_graph.addData(value.position[0], 0)
        self.j0_graph.addData(value.velocity[0], 1)
        self.j0_graph.addData(value.effort[0] * self.j0_graphs_effort_scale, 2)

        self.j1_graph.addData(value.position[1], 0)
        self.j1_graph.addData(value.velocity[1], 1)
        self.j1_graph.addData(value.effort[1] * self.j0_graphs_effort_scale, 2)

        self.j2_graph.addData(value.position[2], 0)
        self.j2_graph.addData(value.velocity[2], 1)
        self.j2_graph.addData(value.effort[2] * self.j0_graphs_effort_scale, 2)

        self.j3_graph.addData(value.position[3], 0)
        self.j3_graph.addData(value.velocity[3], 1)
        self.j3_graph.addData(value.effort[3] * self.j0_graphs_effort_scale, 2)

        self.j4_graph.addData(value.position[4], 0)
        self.j4_graph.addData(value.velocity[4], 1)
        self.j4_graph.addData(value.effort[4] * self.j0_graphs_effort_scale, 2)

        self.j5_graph.addData(value.position[5], 0)
        self.j5_graph.addData(value.velocity[5], 1)
        self.j5_graph.addData(value.effort[5] * self.j0_graphs_effort_scale, 2)

        self.j6_graph.addData(value.position[6], 0)
        self.j6_graph.addData(value.velocity[6], 1)
        self.j6_graph.addData(value.effort[6] * self.j0_graphs_effort_scale, 2)

        self.j7_graph.addData(value.position[7], 0)
        self.j7_graph.addData(value.velocity[7], 1)
        self.j7_graph.addData(value.effort[7] * self.j0_graphs_effort_scale, 2)

        self.j8_graph.addData(value.position[8], 0)
        self.j8_graph.addData(value.velocity[8], 1)
        self.j8_graph.addData(value.effort[8] * self.j0_graphs_effort_scale, 2)

        self.j9_graph.addData(value.position[9], 0)
        self.j9_graph.addData(value.velocity[9], 1)
        self.j9_graph.addData(value.effort[9] * self.j0_graphs_effort_scale, 2)

        self.j10_graph.addData(value.position[10], 0)
        self.j10_graph.addData(value.velocity[10], 1)
        self.j10_graph.addData(value.effort[10] * self.j0_graphs_effort_scale, 2)

        self.j11_graph.addData(value.position[11], 0)
        self.j11_graph.addData(value.velocity[11], 1)
        self.j11_graph.addData(value.effort[11] * self.j0_graphs_effort_scale, 2)

        self.j12_graph.addData(value.position[12], 0)
        self.j12_graph.addData(value.velocity[12], 1)
        self.j12_graph.addData(value.effort[12] * self.j0_graphs_effort_scale, 2)

        self.j13_graph.addData(value.position[13], 0)
        self.j13_graph.addData(value.velocity[13], 1)
        self.j13_graph.addData(value.effort[13] * self.j0_graphs_effort_scale, 2)

        self.j14_graph.addData(value.position[14], 0)
        self.j14_graph.addData(value.velocity[14], 1)
        self.j14_graph.addData(value.effort[14] * self.j0_graphs_effort_scale, 2)

        self.j15_graph.addData(value.position[15], 0)
        self.j15_graph.addData(value.velocity[15], 1)
        self.j15_graph.addData(value.effort[15] * self.j0_graphs_effort_scale, 2)

        self.j16_graph.addData(value.position[16], 0)
        self.j16_graph.addData(value.velocity[16], 1)
        self.j16_graph.addData(value.effort[16] * self.j0_graphs_effort_scale, 2)

        self.j17_graph.addData(value.position[17], 0)
        self.j17_graph.addData(value.velocity[17], 1)
        self.j17_graph.addData(value.effort[17] * self.j0_graphs_effort_scale, 2)

        self.j18_graph.addData(value.position[18], 0)
        self.j18_graph.addData(value.velocity[18], 1)
        self.j18_graph.addData(value.effort[18] * self.j0_graphs_effort_scale, 2)

        self.j19_graph.addData(value.position[19], 0)
        self.j19_graph.addData(value.velocity[19], 1)
        self.j19_graph.addData(value.effort[19] * self.j0_graphs_effort_scale, 2)

        self.j20_graph.addData(value.position[20], 0)
        self.j20_graph.addData(value.velocity[20], 1)
        self.j20_graph.addData(value.effort[20] * self.j0_graphs_effort_scale, 2)

        self.j21_graph.addData(value.position[21], 0)
        self.j21_graph.addData(value.velocity[21], 1)
        self.j21_graph.addData(value.effort[21] * self.j0_graphs_effort_scale, 2)

        self.j22_graph.addData(value.position[22], 0)
        self.j22_graph.addData(value.velocity[22], 1)
        self.j22_graph.addData(value.effort[22] * self.j0_graphs_effort_scale, 2)

        self.j23_graph.addData(value.position[23], 0)
        self.j23_graph.addData(value.velocity[23], 1)
        self.j23_graph.addData(value.effort[23] * self.j0_graphs_effort_scale, 2)


    def mech_stat_cb(self, value):
        #print(value.joint_statistics[2])
        self.graph_three.addData(value.joint_statistics[2].position, 0)
        self.graph_three.addData(value.joint_statistics[2].velocity, 1)
        self.graph_three.addData(value.joint_statistics[2].measured_effort, 2)

    def p_val_cb(self, value):
        # print("process_value: " + str(value.process_value))
        self.graph_one.addData(value.set_point, 0)
        self.graph_one.addData(value.process_value, 1)
        self.graph_one.addData(value.process_value_dot, 2)
        self.graph_one.addData(value.error, 3)
        self.graph_one.addData(value.command, 4)



    def p_val_dot_cb(self, value):
        # print("process_value_dot: " + str(value.process_value_dot))
        #self.graph_two.addData(value.process_value_dot, 0)
        #self.graph_one.addData(value.process_value_dot, 1)
        self.graph_two.addData(value.set_point, 0)
        self.graph_two.addData(value.process_value, 1)
        self.graph_two.addData(value.process_value_dot, 2)
        self.graph_two.addData(value.error, 3)
        self.graph_two.addData(value.command, 4)

    def create_scene_plugin(self):
        package_path = rospkg.RosPack().get_path('sr_data_visualization')
        rviz_config_approach = package_path + "/uis/handescene.rviz"

        reader = rviz.YamlConfigReader()

        # Configuring approach window
        config_approach = rviz.Config()
        reader.readFile(config_approach, rviz_config_approach)
        self.frame_scene = rviz.VisualizationFrame()
        self.frame_scene.setSplashPath("")
        self.frame_scene.initialize()
        self.frame_scene.setMenuBar(None)
        self.frame_scene.setStatusBar(None)
        self.frame_scene.setHideButtonVisibility(False)
        self.frame_scene.load(config_approach)

        scene_layout = self._widget.findChild(QVBoxLayout, "scene_layout")
        scene_layout.addWidget(self.frame_scene)
        #self.load_scene_file("empty")

    def load_scene_file(self, scene_name):
        try:
            scenes_path = "`rospack find sr_data_visualization`/scenes/" + scene_name + ".scene"
            p = subprocess.Popen(['rosrun moveit_ros_planning moveit_publish_scene_from_text {}'.format(scenes_path)],
                                 shell=True)
        except rospy.ROSException as e:
            rospy.logerr("There was an error loading the scene: ", scene_name)
            rospy.logerr(e)
            return

    def init_widget_children(self):

        self.plan_time_layout = self._widget.findChild(QVBoxLayout, "pid_clipped_layout")
        self.finger_position_layout = self._widget.findChild(QVBoxLayout, "pid_layout")
        self.pos_vel_eff_layout = self._widget.findChild(QVBoxLayout, "pos_vel_eff_layout")

        self.j0_layout = self._widget.findChild(QVBoxLayout, "j0_layout")
        self.j1_layout = self._widget.findChild(QVBoxLayout, "j1_layout")
        self.j2_layout = self._widget.findChild(QVBoxLayout, "j2_layout")
        self.j3_layout = self._widget.findChild(QVBoxLayout, "j3_layout")
        self.j4_layout = self._widget.findChild(QVBoxLayout, "j4_layout")
        self.j5_layout = self._widget.findChild(QVBoxLayout, "j5_layout")
        self.j6_layout = self._widget.findChild(QVBoxLayout, "j6_layout")
        self.j7_layout = self._widget.findChild(QVBoxLayout, "j7_layout")
        self.j8_layout = self._widget.findChild(QVBoxLayout, "j8_layout")

        self.j9_layout = self._widget.findChild(QVBoxLayout, "j9_layout")
        self.j10_layout = self._widget.findChild(QVBoxLayout, "j10_layout")
        self.j11_layout = self._widget.findChild(QVBoxLayout, "j11_layout")
        self.j12_layout = self._widget.findChild(QVBoxLayout, "j12_layout")
        self.j13_layout = self._widget.findChild(QVBoxLayout, "j13_layout")
        self.j14_layout = self._widget.findChild(QVBoxLayout, "j14_layout")
        self.j15_layout = self._widget.findChild(QVBoxLayout, "j15_layout")
        self.j16_layout = self._widget.findChild(QVBoxLayout, "j16_layout")
        self.j17_layout = self._widget.findChild(QVBoxLayout, "j17_layout")

        self.j18_layout = self._widget.findChild(QVBoxLayout, "j18_layout")
        self.j19_layout = self._widget.findChild(QVBoxLayout, "j19_layout")
        self.j20_layout = self._widget.findChild(QVBoxLayout, "j20_layout")
        self.j21_layout = self._widget.findChild(QVBoxLayout, "j21_layout")
        self.j22_layout = self._widget.findChild(QVBoxLayout, "j22_layout")
        self.j23_layout = self._widget.findChild(QVBoxLayout, "j23_layout")


    def init_plots(self):
        self.create_graph(self.plan_time_layout)
        #self.create_graph(self.finger_position_layout)

    def create_graph(self, layout):

        figcanvas = CustomFigCanvas()
        figcanvas.setParent(self._widget)
        layout.addWidget(figcanvas)



    def create_menu_bar(self):
        self._widget.myQMenuBar = QMenuBar(self._widget)
        fileMenu = self._widget.myQMenuBar.addMenu('&File')


class CustomFigCanvas(FigureCanvas, TimedAnimation):
###https://stackoverflow.com/questions/36665850/matplotlib-animation-inside-your-own-pyqt4-gui
    def __init__(self, num_lines, colour = [], ymin = -1, ymax = 1, legends = []):
        self.num_lines = num_lines
        self.addedDataArray = []
        n = 0
        while n < self.num_lines:
            addedData = []
            self.addedDataArray.append(addedData)
            n = n + 1

        print(matplotlib.__version__)

        # The data
        self.xlim = 200
        self.n = np.linspace(0, self.xlim - 1, self.xlim)
        self.y = []
        n = 0
        while n < self.num_lines:
            self.y.append((self.n * 0.0) + 50)
            n = n + 1


        # The window
        self.fig = Figure(figsize=(3,3), dpi=100)
        self.ax1 = self.fig.add_subplot(111)

        # self.ax1 settings
        self.ax1.set_xlabel('time')
        self.ax1.set_ylabel('raw data')
        self.ax1.legend
        i = 0

        self.line = []
        self.line_head = []
        self.line_tail = []
        print("line: ", self.num_lines)
        while i < self.num_lines:
            print("i: ",i)
            self.line.append(Line2D([], [], color=colour[i]))
            self.line_tail.append(Line2D([], [], color='red', linewidth=2))
            self.line_head.append(Line2D([], [], color='red', marker='o', markeredgecolor='r'))
            self.ax1.add_line(self.line[i])
            self.ax1.add_line(self.line_tail[i])
            self.ax1.add_line(self.line_head[i])
            self.ax1.set_xlim(0, self.xlim - 1)
            self.ax1.set_ylim(ymin, ymax)
            i = i + 1

        self.ax1.legend(self.line, legends, bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=self.num_lines, mode="expand", borderaxespad=0., prop={'size': 7})

        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval = 50, blit = True)


    def new_frame_seq(self):
        return iter(range(self.n.size))

    def _init_draw(self):
        i = 0
        while i < self.num_lines:
            lines = [self.line[i], self.line_tail[i], self.line_head[i]]
            for l in lines:
                l.set_data([], [])
            i = i + 1

    def addData(self, value, index):
        self.addedDataArray[index].append(value)
        #self.addedData.append(value)

    def zoomIn(self, value):
        bottom = self.ax1.get_ylim()[0]
        top = self.ax1.get_ylim()[1]
        bottom += value
        top -= value
        self.ax1.set_ylim(bottom,top)
        self.draw()


    def _step(self, *args):
        # Extends the _step() method for the TimedAnimation class.
        try:
            TimedAnimation._step(self, *args)
        except Exception as e:
            self.abc += 1
            print(str(self.abc))
            TimedAnimation._stop(self)
            pass

    def _draw_frame(self, framedata):
        margin = 2
        i = 0
        while i < self.num_lines:
            while(len(self.addedDataArray[i]) > 0):
                self.y[i] = np.roll(self.y[i], -1)
                self.y[i][-1] = self.addedDataArray[i][0]
                del(self.addedDataArray[i][0])
            i = i + 1
        i = 0
        while i < self.num_lines:
            self.line[i].set_data(self.n[ 0 : self.n.size - margin ], self.y[i][ 0 : self.n.size - margin ])
            self.line_tail[i].set_data(np.append(self.n[-10:-1 - margin], self.n[-1 - margin]), np.append(self.y[i][-10:-1 - margin], self.y[i][-1 - margin]))
            self.line_head[i].set_data(self.n[-1 - margin], self.y[i][-1 - margin])
            self._drawn_artists = []
            for l in self.line:
                self._drawn_artists.append(l)
            for l in self.line_tail:
                self._drawn_artists.append(l)
            for l in self.line_head:
                self._drawn_artists.append(l)
            i = i + 1


if __name__ == "__main__":
    rospy.init_node("hand_e_visualizer")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrDataVisualizer(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
