#!/usr/bin/env python

import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib import __version__ as matplotlibversion

import signal
import rospy
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

    plt.axis([0, 10, 0, 1])

    for i in range(10):
        y = np.random.random()
        plt.scatter(i, y)
        plt.pause(0.05)

    plt.show()

    def init_widget_children(self):

        self.plan_time_layout = self._widget.findChild(QVBoxLayout, "plan_time_layout")

        self.perquery_plan_time_layout = self._widget.findChild(QVBoxLayout, "perquery_plan_time_layout")

        self.perquery_plan_time_layout_2 = self._widget.findChild(QVBoxLayout, "perquery_plan_time_layout_2")


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

    def load_database(self):
        path_to_db = None
        db_to_be_loaded = self.dbs_combo_box.currentText()
        for db in self.available_databases:
            if db_to_be_loaded == db['rel_path']:
                path_to_db = db['full_path']
                break
        if path_to_db is not None:
            self.connect_to_database(path_to_db)
            self.get_planners_list()
            self.get_queries_list()
            self.update_data_display()

    def load_bench_conf(self):
        self.bench_config_combo_box.clear()
        directory = rospkg.RosPack().get_path('sr_moveit_planner_benchmarking') + "/experiments/benchmark_configs/"
        for root, dirs, files in os.walk(directory):
            for file in files:
                if file.endswith(".yaml"):
                    full_path = os.path.join(root, file)
                    rel_path = os.path.relpath(full_path, directory)
                    self.bench_config_combo_box.addItem(rel_path)
                    self.available_benchmarks.append({'rel_path': rel_path, 'full_path': full_path})

    def create_menu_bar(self):
        self._widget.myQMenuBar = QMenuBar(self._widget)
        fileMenu = self._widget.myQMenuBar.addMenu('&File')

    def plot_attribute(self, cur, planners, attribute, typename, layout):
        labels = []
        measurements = []
        total_per_planner = []

        for planner in planners:
            cur.execute('SELECT %s FROM runs WHERE plannerid = %s AND %s IS NOT NULL'
                        % (attribute, planner[0], attribute))
            measurement = [t[0] for t in cur.fetchall() if t[0] != None]
            cur.execute('SELECT count(*) FROM runs WHERE plannerid = %s'
                        % (planner[0]))
            total_per_planner.append(cur.fetchone()[0])

            labels.append(planner[1])
            measurements.append(measurement)

        if len(measurements) == 0:
            print('Skipping "%s": no available measurements' % attribute)
            return

        self.plot_measurements(measurements, total_per_planner, planners, attribute, typename, labels, layout)

    def plot_measurements(self, measurements, total_per_planner, planners, attribute, typename, labels, layout):
        plt.clf()

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
            measurementsPercentage = [sum(m) * 100. / total_per_planner[counter] for counter, m in
                                      enumerate(measurements)]
            ind = range(len(measurements))
            ax.bar(ind, measurementsPercentage, width)
            plt.setp(ax, xticks=[x + width / 2. for x in ind])
            ax.set_ylim([0, 100])
            ax.set_xlim([0, len(planners)])
        else:
            if int(matplotlibversion.split('.')[0]) < 1:
                ax.boxplot(measurements, notch=0, sym='r+', vert=1, whis=1.5)
            else:
                ax.boxplot(measurements, notch=0, sym='r+', vert=1, whis=1.5, bootstrap=1000)

        xtickNames = plt.setp(ax, xticklabels=labels)
        plt.setp(xtickNames, rotation=90)
        for tick in ax.xaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(7)
        for tick in ax.yaxis.get_major_ticks():  # shrink the font size of the y tick labels
            tick.label.set_fontsize(7)
        fig.subplots_adjust(bottom=0.32, top=0.90, left=0.08, right=0.98)
        ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)

        if "clearance" in attribute:
            ax.ticklabel_format(style='sci', axis='y', scilimits=(-3, 4), useLocale=True)
            ax.yaxis.offsetText.set_fontsize(7)

        self.clearLayout(layout)
        layout.addWidget(figcanvas)

    def plot_attribute_per_query_per_query(self, cur, query, planners, attribute, typename, layout):
        # Plotting each query results in a graph with the planners in x axis
        experiment_id = [given_query[0] for given_query in self.queries if given_query[1] == query][0]

        labels = []
        measurements = []
        total_per_planner = []

        for planner in planners:
            cur.execute('SELECT %s FROM runs WHERE experimentid=%s AND plannerid = %s AND %s IS NOT NULL'
                        % (attribute, experiment_id, planner[0], attribute))
            measurement = [t[0] for t in cur.fetchall() if t[0] != None]
            cur.execute('SELECT count(*) FROM runs WHERE experimentid=%s AND plannerid = %s'
                        % (experiment_id, planner[0]))
            total_per_planner.append(cur.fetchone()[0])

            labels.append(planner[1])
            measurements.append(measurement)

        if len(measurements) == 0:
            print('Skipping "%s": no available measurements' % attribute)
            return

        self.plot_measurements(measurements, total_per_planner, planners, attribute, typename, labels, layout)

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

    def plot_statistics(self):
        self.c.execute('PRAGMA table_info(runs)')
        colInfo = self.c.fetchall()[3:]

        for col in colInfo:

            if "time" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.plan_time_layout)


    def plot_statistics_per_query_per_planner(self, planner):
        self.c.execute('PRAGMA table_info(runs)')
        colInfo = self.c.fetchall()[3:]

        for col in colInfo:

            if "time" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_plan_time_layout)

        self.queries_legend.clear()
        for query in self.queries:
            self.queries_legend.append('Query {}: {}'.format(query[0], query[1]))

    def plot_statistics_per_query_per_query(self, query):
        self.c.execute('PRAGMA table_info(runs)')
        colInfo = self.c.fetchall()[3:]

        for col in colInfo:

            if "time" == col[1]:
                self.plot_attribute_per_query_per_query(self.c, query, self.planners, col[1], col[2],
                                                        self.perquery_plan_time_layout_2)



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



if __name__ == "__main__":
    rospy.init_node("hand_e_visualizer")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrMoveitPlannerBenchmarksVisualizer(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
