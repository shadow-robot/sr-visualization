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

        #self.init_plots()
        self.graph_one = CustomFigCanvas('green')
        self.graph_two = CustomFigCanvas('blue')
        #self.create_graph(self.plan_time_layout)

        self.sub = rospy.Subscriber('sh_rh_ffj0_position_controller/state', JointControllerState,
                                    self.p_val_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('sh_rh_ffj0_position_controller/state', JointControllerState,
                                    self.p_val_dot_cb,
                                    queue_size=1)

        self.graph_one.setParent(self._widget)
        #self.graph_two = CustomFigCanvas()
        #self.graph_three = CustomFigCanvas()


        self.plan_time_layout.addWidget(self.graph_one)
        self.finger_position_layout.addWidget(self.graph_two)

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




    def p_val_cb(self, value):
        #print("process_value: " + str(value.process_value))
        self.graph_one.addData(value.process_value)

    def p_val_dot_cb(self, value):
        #print("process_value_dot: " + str(value.process_value_dot))
        self.graph_two.addData(value.process_value_dot)


    def init_widget_children(self):

        self.plan_time_layout = self._widget.findChild(QVBoxLayout, "plan_time_layout")
        self.finger_position_layout = self._widget.findChild(QVBoxLayout, "finger_position_layout")

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
    def __init__(self, Color):

        self.addedData = []
        print(matplotlib.__version__)

        # The data
        self.xlim = 200
        self.n = np.linspace(0, self.xlim - 1, self.xlim)
        self.y = (self.n * 0.0) + 50

        # The window
        self.fig = Figure(figsize=(5,5), dpi=100)
        self.ax1 = self.fig.add_subplot(111)



        # self.ax1 settings
        self.ax1.set_xlabel('time')
        self.ax1.set_ylabel('raw data')
        self.line1 = Line2D([], [], color=Color)
        self.line1_tail = Line2D([], [], color='red', linewidth=2)
        self.line1_head = Line2D([], [], color='red', marker='o', markeredgecolor='r')
        self.ax1.add_line(self.line1)
        self.ax1.add_line(self.line1_tail)
        self.ax1.add_line(self.line1_head)
        self.ax1.set_xlim(0, self.xlim - 1)
        self.ax1.set_ylim(-4, 4)


        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval = 50, blit = True)



    def new_frame_seq(self):
        return iter(range(self.n.size))

    def _init_draw(self):
        lines = [self.line1, self.line1_tail, self.line1_head]
        for l in lines:
            l.set_data([], [])

    def addData(self, value):
        self.addedData.append(value)

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
        while(len(self.addedData) > 0):
            self.y = np.roll(self.y, -1)
            self.y[-1] = self.addedData[0]
            del(self.addedData[0])


        self.line1.set_data(self.n[ 0 : self.n.size - margin ], self.y[ 0 : self.n.size - margin ])
        self.line1_tail.set_data(np.append(self.n[-10:-1 - margin], self.n[-1 - margin]), np.append(self.y[-10:-1 - margin], self.y[-1 - margin]))
        self.line1_head.set_data(self.n[-1 - margin], self.y[-1 - margin])
        self._drawn_artists = [self.line1, self.line1_tail, self.line1_head]



if __name__ == "__main__":
    rospy.init_node("hand_e_visualizer")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrDataVisualizer(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
