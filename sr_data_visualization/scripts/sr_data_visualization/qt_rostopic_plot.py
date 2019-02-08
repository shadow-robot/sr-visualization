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
import matplotlib.pyplot as plt
from matplotlib import __version__ as matplotlibversion

from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D


import signal
import rospy
from control_msgs.msg import JointControllerState
import os
import rospkg
import sqlite3
import rviz
import subprocess




class SrMoveitPlannerBenchmarksVisualizer(Plugin):
    def __init__(self, context):
        super(SrMoveitPlannerBenchmarksVisualizer, self).__init__(context)
        self.setObjectName("SrMoveitPlannerBenchmarksVisualizer")
        self._widget = QWidget()
        self.create_menu_bar()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_data_visualization'), 'uis', 'hand-e_visualizer.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Moveit Planner Benchmarks")
        self.init_widget_children()
        #self.init_plots()
        self.graph_one = CustomFigCanvas()
        self.graph_two = CustomFigCanvas()
        self.graph_three = CustomFigCanvas()
        #self.create_graph(self.plan_time_layout)

        self.sub = rospy.Subscriber('sh_rh_ffj0_position_controller/state', JointControllerState,
                                    self.addData_callbackFunc,
                                    queue_size=1)

        self.graph_one.setParent(self._widget)
        self.graph_two = CustomFigCanvas()
        self.graph_three = CustomFigCanvas()


        self.plan_time_layout.addWidget(self.graph_one)
        self.finger_position_layout.addWidget(self.graph_two)




    def addData_callbackFunc(self, value):
        print("data: " + str(value.process_value))
        self.graph_one.addData(value.process_value)
        self.graph_two.addData(value.process_value)


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

    def destruct(self):
        self._widget = None
        rospy.loginfo("Closing planner benchmarks visualizer")

    def update_data_display(self):
        self.set_planners_combobox()
        self.set_queries_combobox()
        self.plot_statistics()
        self.change_plots_per_query_per_planner()
        self.change_plots_per_query_per_query()
        self.set_experiments_info()
        scene_name = self.find_scene()
        self.scene_label.setText(scene_name)
        self.load_scene_file(scene_name)


    def create_menu_bar(self):
        self._widget.myQMenuBar = QMenuBar(self._widget)
        fileMenu = self._widget.myQMenuBar.addMenu('&File')

    # def plot_measurements(self, measurements, total_per_planner, planners, attribute, typename, labels, layout):
    #     plt.clf()
    #
    #     width = 5
    #     height = 4
    #     dpi = 100
    #     fig = Figure(figsize=(width, height), dpi=dpi, facecolor=(1.0, 1.0, 1.0, 1.0))
    #     ax = fig.add_subplot(111)
    #
    #     figcanvas = CustomFigCanvas()
    #     figcanvas.setParent(self._widget)
    #     FigureCanvas.setSizePolicy(figcanvas, QSizePolicy.Expanding, QSizePolicy.Expanding)
    #     FigureCanvas.updateGeometry(figcanvas)
    #
    #
    #     if int(matplotlibversion.split('.')[0]) < 1:
    #         ax.boxplot(measurements, notch=0, sym='r+', vert=1, whis=1.5)
    #     else:
    #         ax.boxplot(measurements, notch=0, sym='r+', vert=1, whis=1.5, bootstrap=1000)
    #
    #     xtickNames = plt.setp(ax, xticklabels=labels)
    #     plt.setp(xtickNames, rotation=90)
    #     for tick in ax.xaxis.get_major_ticks():  # shrink the font size of the x tick labels
    #         tick.label.set_fontsize(7)
    #     for tick in ax.yaxis.get_major_ticks():  # shrink the font size of the y tick labels
    #         tick.label.set_fontsize(7)
    #     fig.subplots_adjust(bottom=0.32, top=0.90, left=0.08, right=0.98)
    #     ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    #
    #     if "clearance" in attribute:
    #         ax.ticklabel_format(style='sci', axis='y', scilimits=(-3, 4), useLocale=True)
    #         ax.yaxis.offsetText.set_fontsize(7)
    #
    #     self.clearLayout(layout)
    #     layout.addWidget(figcanvas)

    def plot_attribute_per_query(self, cur, planner, attribute, typename, layout):
        # Plotting for the selected planner results for each query
        measurements = []
        cur.execute('SELECT %s FROM runs WHERE plannerid = %s'
                    % (attribute, planner[0]))
        measurement_including_nan = [t[0] for t in cur.fetchall()]

        if 0 == len(measurement_including_nan):
            rospy.logwarn("No measurements for {} available!".format(attribute))
            self.clearLayout(layout)
            return

        cur.execute('SELECT experimentid FROM runs WHERE plannerid = %s' % (planner[0]))
        queryid_to_run_mapping = [t[0] for t in cur.fetchall()]

        per_query_runcount = []
        for query in self.queries:
            per_query_runcount.append(queryid_to_run_mapping.count(query[0]))

        matrix_measurements_with_nans = []
        for runcount in per_query_runcount:
            matrix_measurements_with_nans.append(measurement_including_nan[0: runcount])
            del measurement_including_nan[0: runcount]

        # Remove NaNs from the measurement array
        matrix_measurements = []
        for per_query_result in matrix_measurements_with_nans:
            matrix_measurements.append([x for x in per_query_result if x is not None])

        plt.clf()
        # GUI
        width = 5
        height = 4
        dpi = 100
        fig = Figure(figsize=(width, height), dpi=dpi, facecolor=(1.0, 1.0, 1.0, 1.0))
        ax = fig.add_subplot(111)

        figcanvas = FigureCanvas(fig)
        figcanvas.setParent(self._widget)
        FigureCanvas.setSizePolicy(figcanvas, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(figcanvas)

        if typename == 'BOOLEAN':
            width = .5
            measurements_percentage = []
            missing_measurements = []
            missing_measurements_index = []
            for i, m in enumerate(matrix_measurements):
                if 0 == len(m):
                    measurements_percentage.append(0)
                    missing_measurements.append(50)
                    missing_measurements_index.append(i + width / 2)
                else:
                    measurements_percentage.append(sum(m) * 100 / per_query_runcount[i])
            idx = range(len(measurements_percentage))
            ax.bar(idx, measurements_percentage, width)
            ax.scatter(missing_measurements_index, missing_measurements, color='r', marker='x')
            plt.setp(ax, xticks=[x + width / 2 for x in idx], xticklabels=[x + 1 for x in idx])
            ax.set_ylim([0, 100])
            ax.set_xlim([0, len(matrix_measurements)])
        else:
            if int(matplotlibversion.split('.')[0]) < 1:
                ax.boxplot(measurements, notch=0, sym='r+', vert=1, whis=1.5)
            else:
                ax.boxplot(matrix_measurements, notch=0, sym='r+', vert=1, whis=1.5, bootstrap=1000)

        for tick in ax.xaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(7)
        for tick in ax.yaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(7)

        fig.subplots_adjust(bottom=0.1, top=0.90, left=0.08, right=0.98)
        ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)

        if "clearance" in attribute:
            ax.ticklabel_format(style='sci', axis='y', scilimits=(-3, 4), useLocale=True)
            ax.yaxis.offsetText.set_fontsize(7)

        self.clearLayout(layout)
        layout.addWidget(figcanvas)



    def clearLayout(self, layout):
        for i in reversed(range(layout.count())):
            layout.itemAt(i).widget().setParent(None)

    def get_planners_list(self):
        self.c.execute('PRAGMA FOREIGN_KEYS = ON')
        self.c.execute('SELECT id, name FROM plannerConfigs')
        planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '').replace('kConfigDefault', ''))
                    for t in self.c.fetchall()]
        self.planners = sorted(planners, key=lambda a: a[1])

    def get_queries_list(self):
        self.c.execute('SELECT id, name FROM experiments')
        self.queries = [(q[0], q[1]) for q in self.c.fetchall()]

    def set_planners_combobox(self):
        self.planners_combo_box.blockSignals(True)
        self.planners_combo_box.clear()
        for planner in self.planners:
            self.planners_combo_box.addItem(planner[1])
        self.planners_combo_box.blockSignals(False)



''' End Class '''


class CustomFigCanvas(FigureCanvas, TimedAnimation):

    def __init__(self):

        self.addedData = []
        print(matplotlib.__version__)

        # The data
        self.xlim = 200
        self.n = np.linspace(0, self.xlim - 1, self.xlim)
        a = []
        b = []
        a.append(2.0)
        a.append(4.0)
        a.append(2.0)
        b.append(4.0)
        b.append(3.0)
        b.append(4.0)
        self.y = (self.n * 0.0) + 50

        # The window
        self.fig = Figure(figsize=(5,5), dpi=100)
        self.ax1 = self.fig.add_subplot(111)


        # self.ax1 settings
        self.ax1.set_xlabel('time')
        self.ax1.set_ylabel('raw data')
        self.line1 = Line2D([], [], color='blue')
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
    planner_benchmarking_gui = SrMoveitPlannerBenchmarksVisualizer(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
