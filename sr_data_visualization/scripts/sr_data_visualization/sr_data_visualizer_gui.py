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
from sr_robot_msgs.msg import BiotacAll
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import os
import rospkg
import rviz
import subprocess



class SrDataVisualizer(Plugin):
    def __init__(self, context):
        super(SrDataVisualizer, self).__init__(context)
        self.setObjectName("SrDataVisualizer")
        self._widget = QWidget()
        #self.create_menu_bar()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_data_visualization'), 'uis', 'hand-e_visualizer.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Hand E Visualizer")
        tab_widg = self._widget.findChild(QTabWidget, "tabWidget")
        print(tab_widg)
        tab_widg.currentChanged.connect(self.tab_change)
        self.init_widget_children()
        self.create_scene_plugin()

        self.j0_graphs_scale = 3.14159
        self.pid_output_scale = 0.013333333

        self.j0_graphs_effort_scale = self.j0_graphs_scale/600
        self.create_graphs()
        self.create_subscribers()
        self.attach_graphs()

    def tab_change(self, val):
        print("triggered")
        print(val)
        self._widget.update()

    def create_graphs(self):
        #Overview page graphs
        self.palm_extras_graph = CustomFigCanvas(10, ['red', 'cyan', 'green', 'purple', 'yellow', 'blue', 'green', 'pink', 'red', 'magenta'], 0, 36, ['accel x', 'accel y', 'accel z', 'gyro x', 'gyro y', 'gyro z', 'ADC0', 'ADC1', 'ADC2', 'ADC3'], 2)
        self.effort_graph = CustomFigCanvas(2, ['red', 'cyan', ], -100, 100, ['Commanded Effort', 'Measured Effort'])
        self.biotac_overview_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 1000, ['PAC0', 'PAC1', 'PDC', 'TAC', 'TDC'])
        self.pid_clipped_graph = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4, ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -300, 300, ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pos_vel_eff_graph = CustomFigCanvas(3, ['red', 'blue', 'green'], -100, 100, ['Position', 'Velocity', 'Effort'])

        #Biotac page graphs
        self.biotac_0_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 1000,
                                              ['PAC0', 'PAC1', 'PDC', 'TAC', 'TDC'])
        self.biotac_2_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 1000,
                                              ['PAC0', 'PAC1', 'PDC', 'TAC', 'TDC'])
        self.biotac_1_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 1000,
                                              ['PAC0', 'PAC1', 'PDC', 'TAC', 'TDC'])
        self.biotac_3_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 1000,
                                              ['PAC0', 'PAC1', 'PDC', 'TAC', 'TDC'])
        self.biotac_4_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 1000,
                                              ['PAC0', 'PAC1', 'PDC', 'TAC', 'TDC'])
        self.biotacs_tac_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 34000,
                                                 ['biotac_0', 'biotac_1', 'biotac_2', 'biotac_3', 'biotac_4'])
        self.biotacs_pdc_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 20000,
                                                 ['biotac_0', 'biotac_1', 'biotac_2', 'biotac_3', 'biotac_4'])
        self.biotacs_pac_graph = CustomFigCanvas(5, ['red', 'cyan', 'green', 'purple', 'blue'], 0, 1000,
                                                 ['biotac_0', 'biotac_1', 'biotac_2', 'biotac_3', 'biotac_4'])

        j0_graphs_scale = self.j0_graphs_scale
        #All joints page graphs
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

        self.pid_graph_j0 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j1 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j2 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j3 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j4 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j5 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j6 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j7 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j8 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j9 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j10 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j11 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j12 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j13 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j14 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j15 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j16 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j17 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j18 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j19 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j20 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j21 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                         ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j22 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                          ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])
        self.pid_graph_j23 = CustomFigCanvas(5, ['red', 'blue', 'green', 'purple', 'cyan'], -4, 4,
                                          ['Setpoint', 'Input', 'dInput/dt', 'Error', 'Output'])

    def create_subscribers(self):
        self.sub = rospy.Subscriber('sh_rh_ffj0_position_controller/state', JointControllerState, self.pid_graphs_cb, queue_size=1)
        self.sub = rospy.Subscriber('mechanism_statistics', MechanismStatistics, self.mech_stat_cb, queue_size=1)
        self.sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb, queue_size=1)
        self.sub = rospy.Subscriber('/rh/palm_extras', Float64MultiArray, self.palm_extras_cb, queue_size=1)
        self.sub = rospy.Subscriber('/rh/tactile', BiotacAll, self.biotac_all_cb, queue_size=1)
        #self.graph_one.setParent(self._widget)
        self.sub = rospy.Subscriber('sh_rh_ffj0_position_controller/state', JointControllerState, self.pid_graphs_cb, queue_size=1)

        self.sub = rospy.Subscriber('/sh_rh_ffj0_position_controller/state', JointControllerState, self.j0_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_ffj3_position_controller/state', JointControllerState, self.j1_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_ffj4_position_controller/state', JointControllerState, self.j2_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_lfj0_position_controller/state', JointControllerState, self.j3_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_lfj3_position_controller/state', JointControllerState, self.j4_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_lfj4_position_controller/state', JointControllerState, self.j5_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_lfj5_position_controller/state', JointControllerState, self.j6_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_mfj0_position_controller/state', JointControllerState, self.j7_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_mfj3_position_controller/state', JointControllerState, self.j8_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_mfj4_position_controller/state', JointControllerState, self.j9_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_rfj0_position_controller/state', JointControllerState, self.j10_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_rfj3_position_controller/state', JointControllerState, self.j11_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_rfj4_position_controller/state', JointControllerState, self.j12_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_thj1_position_controller/state', JointControllerState, self.j13_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_thj2_position_controller/state', JointControllerState, self.j14_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_thj3_position_controller/state', JointControllerState, self.j15_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_thj4_position_controller/state', JointControllerState, self.j16_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_thj5_position_controller/state', JointControllerState, self.j17_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_wrj1_position_controller/state', JointControllerState, self.j18_pid_cb,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/sh_rh_wrj2_position_controller/state', JointControllerState, self.j19_pid_cb,
                                    queue_size=1)

    def attach_graphs(self):
        self.pid_clipped_layout.addWidget(self.pid_clipped_graph)
        self.pid_layout.addWidget(self.pid_graph)
        self.pos_vel_eff_layout.addWidget(self.pos_vel_eff_graph)
        self.palm_extras_layout.addWidget(self.palm_extras_graph)
        self.biotac_0_layout.addWidget(self.biotac_0_graph)
        self.biotac_1_layout.addWidget(self.biotac_1_graph)
        self.biotac_2_layout.addWidget(self.biotac_2_graph)
        self.biotac_3_layout.addWidget(self.biotac_3_graph)
        self.biotac_4_layout.addWidget(self.biotac_4_graph)
        self.biotacs_tac_layout.addWidget(self.biotacs_tac_graph)
        self.biotacs_pac_layout.addWidget(self.biotacs_pac_graph)
        self.biotacs_pdc_layout.addWidget(self.biotacs_pdc_graph)
        self.biotac_overview_layout.addWidget(self.biotac_overview_graph)
        self.effort_layout.addWidget(self.effort_graph)
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

        self.j0_pid_layout.addWidget(self.pid_graph_j0)
        self.j1_pid_layout.addWidget(self.pid_graph_j1)
        self.j2_pid_layout.addWidget(self.pid_graph_j2)
        self.j3_pid_layout.addWidget(self.pid_graph_j3)
        self.j4_pid_layout.addWidget(self.pid_graph_j4)
        self.j5_pid_layout.addWidget(self.pid_graph_j5)
        self.j6_pid_layout.addWidget(self.pid_graph_j6)
        self.j7_pid_layout.addWidget(self.pid_graph_j7)
        self.j8_pid_layout.addWidget(self.pid_graph_j8)
        self.j9_pid_layout.addWidget(self.pid_graph_j9)
        self.j10_pid_layout.addWidget(self.pid_graph_j10)
        self.j11_pid_layout.addWidget(self.pid_graph_j11)
        self.j12_pid_layout.addWidget(self.pid_graph_j12)
        self.j13_pid_layout.addWidget(self.pid_graph_j13)
        self.j14_pid_layout.addWidget(self.pid_graph_j14)
        self.j15_pid_layout.addWidget(self.pid_graph_j15)
        self.j16_pid_layout.addWidget(self.pid_graph_j16)
        self.j17_pid_layout.addWidget(self.pid_graph_j17)
        self.j18_pid_layout.addWidget(self.pid_graph_j18)
        self.j19_pid_layout.addWidget(self.pid_graph_j19)

    def biotac_all_cb(self, value):
        self.biotac_0_graph.addData(value.tactiles[0].pac0,0)
        self.biotac_0_graph.addData(value.tactiles[0].pac1, 1)
        self.biotac_0_graph.addData(value.tactiles[0].pdc, 2)
        self.biotac_0_graph.addData(value.tactiles[0].tac, 3)
        self.biotac_0_graph.addData(value.tactiles[0].tdc, 4)

        self.biotac_1_graph.addData(value.tactiles[1].pac0,0)
        self.biotac_1_graph.addData(value.tactiles[1].pac1, 1)
        self.biotac_1_graph.addData(value.tactiles[1].pdc, 2)
        self.biotac_1_graph.addData(value.tactiles[1].tac, 3)
        self.biotac_1_graph.addData(value.tactiles[1].tdc, 4)

        self.biotac_2_graph.addData(value.tactiles[2].pac0,0)
        self.biotac_2_graph.addData(value.tactiles[2].pac1, 1)
        self.biotac_2_graph.addData(value.tactiles[2].pdc, 2)
        self.biotac_2_graph.addData(value.tactiles[2].tac, 3)
        self.biotac_2_graph.addData(value.tactiles[2].tdc, 4)

        self.biotac_3_graph.addData(value.tactiles[3].pac0,0)
        self.biotac_3_graph.addData(value.tactiles[3].pac1, 1)
        self.biotac_3_graph.addData(value.tactiles[3].pdc, 2)
        self.biotac_3_graph.addData(value.tactiles[3].tac, 3)
        self.biotac_3_graph.addData(value.tactiles[3].tdc, 4)

        self.biotac_4_graph.addData(value.tactiles[4].pac0,0)
        self.biotac_4_graph.addData(value.tactiles[4].pac1, 1)
        self.biotac_4_graph.addData(value.tactiles[4].pdc, 2)
        self.biotac_4_graph.addData(value.tactiles[4].tac, 3)
        self.biotac_4_graph.addData(value.tactiles[4].tdc, 4)

        self.biotacs_tac_graph.addData(value.tactiles[0].tac, 0)
        self.biotacs_tac_graph.addData(value.tactiles[1].tac, 1)
        self.biotacs_tac_graph.addData(value.tactiles[2].tac, 2)
        self.biotacs_tac_graph.addData(value.tactiles[3].tac, 3)
        self.biotacs_tac_graph.addData(value.tactiles[4].tac, 4)

        self.biotacs_pdc_graph.addData(value.tactiles[0].pdc, 0)
        self.biotacs_pdc_graph.addData(value.tactiles[1].pdc, 1)
        self.biotacs_pdc_graph.addData(value.tactiles[2].pdc, 2)
        self.biotacs_pdc_graph.addData(value.tactiles[3].pdc, 3)
        self.biotacs_pdc_graph.addData(value.tactiles[4].pdc, 4)

        self.biotacs_pac_graph.addData(value.tactiles[0].pac0, 0)
        self.biotacs_pac_graph.addData(value.tactiles[1].pac0, 1)
        self.biotacs_pac_graph.addData(value.tactiles[2].pac0, 2)
        self.biotacs_pac_graph.addData(value.tactiles[3].pac0, 3)
        self.biotacs_pac_graph.addData(value.tactiles[4].pac0, 4)

        self.biotac_overview_graph.addData(value.tactiles[0].pac0,0)
        self.biotac_overview_graph.addData(value.tactiles[0].pac1, 1)
        self.biotac_overview_graph.addData(value.tactiles[0].pdc, 2)
        self.biotac_overview_graph.addData(value.tactiles[0].tac, 3)
        self.biotac_overview_graph.addData(value.tactiles[0].tdc, 4)

    def palm_extras_cb(self, value):
        self.palm_extras_graph.addData(value.data[0], 0)
        self.palm_extras_graph.addData(value.data[1], 1)
        self.palm_extras_graph.addData(value.data[2], 2)
        self.palm_extras_graph.addData(value.data[3], 3)
        self.palm_extras_graph.addData(value.data[4], 4)
        self.palm_extras_graph.addData(value.data[5], 5)
        self.palm_extras_graph.addData(value.data[6], 6)
        self.palm_extras_graph.addData(value.data[7], 7)
        self.palm_extras_graph.addData(value.data[8], 8)
        self.palm_extras_graph.addData(value.data[9], 9)

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
        self.pos_vel_eff_graph.addData(value.joint_statistics[2].position, 0)
        self.pos_vel_eff_graph.addData(value.joint_statistics[2].velocity, 1)
        self.pos_vel_eff_graph.addData(value.joint_statistics[2].measured_effort, 2)

        self.effort_graph.addData(value.actuator_statistics[2].last_commanded_effort, 0)
        self.effort_graph.addData(value.actuator_statistics[2].last_measured_effort, 1)

    def j0_pid_cb(self, value):
        self.pid_graph_j0.addData(value.set_point, 0)
        self.pid_graph_j0.addData(value.process_value, 1)
        self.pid_graph_j0.addData(value.process_value_dot, 2)
        self.pid_graph_j0.addData(value.error, 3)
        self.pid_graph_j0.addData(value.command*self.pid_output_scale, 4)

    def j1_pid_cb(self, value):
        self.pid_graph_j1.addData(value.set_point, 0)
        self.pid_graph_j1.addData(value.process_value, 1)
        self.pid_graph_j1.addData(value.process_value_dot, 2)
        self.pid_graph_j1.addData(value.error, 3)
        self.pid_graph_j1.addData(value.command*self.pid_output_scale, 4)

    def j2_pid_cb(self, value):
        self.pid_graph_j2.addData(value.set_point, 0)
        self.pid_graph_j2.addData(value.process_value, 1)
        self.pid_graph_j2.addData(value.process_value_dot, 2)
        self.pid_graph_j2.addData(value.error, 3)
        self.pid_graph_j2.addData(value.command*self.pid_output_scale, 4)

    def j3_pid_cb(self, value):
        self.pid_graph_j3.addData(value.set_point, 0)
        self.pid_graph_j3.addData(value.process_value, 1)
        self.pid_graph_j3.addData(value.process_value_dot, 2)
        self.pid_graph_j3.addData(value.error, 3)
        self.pid_graph_j3.addData(value.command*self.pid_output_scale, 4)

    def j4_pid_cb(self, value):
        self.pid_graph_j4.addData(value.set_point, 0)
        self.pid_graph_j4.addData(value.process_value, 1)
        self.pid_graph_j4.addData(value.process_value_dot, 2)
        self.pid_graph_j4.addData(value.error, 3)
        self.pid_graph_j4.addData(value.command*self.pid_output_scale, 4)

    def j5_pid_cb(self, value):
        self.pid_graph_j5.addData(value.set_point, 0)
        self.pid_graph_j5.addData(value.process_value, 1)
        self.pid_graph_j5.addData(value.process_value_dot, 2)
        self.pid_graph_j5.addData(value.error, 3)
        self.pid_graph_j5.addData(value.command*self.pid_output_scale, 4)

    def j6_pid_cb(self, value):
        self.pid_graph_j6.addData(value.set_point, 0)
        self.pid_graph_j6.addData(value.process_value, 1)
        self.pid_graph_j6.addData(value.process_value_dot, 2)
        self.pid_graph_j6.addData(value.error, 3)
        self.pid_graph_j6.addData(value.command*self.pid_output_scale, 4)

    def j7_pid_cb(self, value):
        self.pid_graph_j7.addData(value.set_point, 0)
        self.pid_graph_j7.addData(value.process_value, 1)
        self.pid_graph_j7.addData(value.process_value_dot, 2)
        self.pid_graph_j7.addData(value.error, 3)
        self.pid_graph_j7.addData(value.command*self.pid_output_scale, 4)

    def j8_pid_cb(self, value):
        self.pid_graph_j8.addData(value.set_point, 0)
        self.pid_graph_j8.addData(value.process_value, 1)
        self.pid_graph_j8.addData(value.process_value_dot, 2)
        self.pid_graph_j8.addData(value.error, 3)
        self.pid_graph_j8.addData(value.command*self.pid_output_scale, 4)

    def j9_pid_cb(self, value):
        self.pid_graph_j9.addData(value.set_point, 0)
        self.pid_graph_j9.addData(value.process_value, 1)
        self.pid_graph_j9.addData(value.process_value_dot, 2)
        self.pid_graph_j9.addData(value.error, 3)
        self.pid_graph_j9.addData(value.command*self.pid_output_scale, 4)

    def j10_pid_cb(self, value):
        self.pid_graph_j10.addData(value.set_point, 0)
        self.pid_graph_j10.addData(value.process_value, 1)
        self.pid_graph_j10.addData(value.process_value_dot, 2)
        self.pid_graph_j10.addData(value.error, 3)
        self.pid_graph_j10.addData(value.command*self.pid_output_scale, 4)

    def j11_pid_cb(self, value):
        self.pid_graph_j11.addData(value.set_point, 0)
        self.pid_graph_j11.addData(value.process_value, 1)
        self.pid_graph_j11.addData(value.process_value_dot, 2)
        self.pid_graph_j11.addData(value.error, 3)
        self.pid_graph_j11.addData(value.command*self.pid_output_scale, 4)

    def j12_pid_cb(self, value):
        self.pid_graph_j12.addData(value.set_point, 0)
        self.pid_graph_j12.addData(value.process_value, 1)
        self.pid_graph_j12.addData(value.process_value_dot, 2)
        self.pid_graph_j12.addData(value.error, 3)
        self.pid_graph_j12.addData(value.command*self.pid_output_scale, 4)

    def j13_pid_cb(self, value):
        self.pid_graph_j13.addData(value.set_point, 0)
        self.pid_graph_j13.addData(value.process_value, 1)
        self.pid_graph_j13.addData(value.process_value_dot, 2)
        self.pid_graph_j13.addData(value.error, 3)
        self.pid_graph_j13.addData(value.command*self.pid_output_scale, 4)

    def j14_pid_cb(self, value):
        self.pid_graph_j14.addData(value.set_point, 0)
        self.pid_graph_j14.addData(value.process_value, 1)
        self.pid_graph_j14.addData(value.process_value_dot, 2)
        self.pid_graph_j14.addData(value.error, 3)
        self.pid_graph_j14.addData(value.command*self.pid_output_scale, 4)

    def j15_pid_cb(self, value):
        self.pid_graph_j15.addData(value.set_point, 0)
        self.pid_graph_j15.addData(value.process_value, 1)
        self.pid_graph_j15.addData(value.process_value_dot, 2)
        self.pid_graph_j15.addData(value.error, 3)
        self.pid_graph_j15.addData(value.command*self.pid_output_scale, 4)

    def j16_pid_cb(self, value):
        self.pid_graph_j16.addData(value.set_point, 0)
        self.pid_graph_j16.addData(value.process_value, 1)
        self.pid_graph_j16.addData(value.process_value_dot, 2)
        self.pid_graph_j16.addData(value.error, 3)
        self.pid_graph_j16.addData(value.command*self.pid_output_scale, 4)

    def j17_pid_cb(self, value):
        self.pid_graph_j17.addData(value.set_point, 0)
        self.pid_graph_j17.addData(value.process_value, 1)
        self.pid_graph_j17.addData(value.process_value_dot, 2)
        self.pid_graph_j17.addData(value.error, 3)
        self.pid_graph_j17.addData(value.command*self.pid_output_scale, 4)

    def j18_pid_cb(self, value):
        self.pid_graph_j18.addData(value.set_point, 0)
        self.pid_graph_j18.addData(value.process_value, 1)
        self.pid_graph_j18.addData(value.process_value_dot, 2)
        self.pid_graph_j18.addData(value.error, 3)
        self.pid_graph_j18.addData(value.command*self.pid_output_scale, 4)

    def j19_pid_cb(self, value):
        self.pid_graph_j19.addData(value.set_point, 0)
        self.pid_graph_j19.addData(value.process_value, 1)
        self.pid_graph_j19.addData(value.process_value_dot, 2)
        self.pid_graph_j19.addData(value.error, 3)
        self.pid_graph_j19.addData(value.command*self.pid_output_scale, 4)

    def j20_pid_cb(self, value):
        self.pid_graph_j20.addData(value.set_point, 0)
        self.pid_graph_j20.addData(value.process_value, 1)
        self.pid_graph_j20.addData(value.process_value_dot, 2)
        self.pid_graph_j20.addData(value.error, 3)
        self.pid_graph_j20.addData(value.command*self.pid_output_scale, 4)

    def j21_pid_cb(self, value):
        self.pid_graph_j21.addData(value.set_point, 0)
        self.pid_graph_j21.addData(value.process_value, 1)
        self.pid_graph_j21.addData(value.process_value_dot, 2)
        self.pid_graph_j21.addData(value.error, 3)
        self.pid_graph_j21.addData(value.command*self.pid_output_scale, 4)

    def j22_pid_cb(self, value):
        self.pid_graph_j22.addData(value.set_point, 0)
        self.pid_graph_j22.addData(value.process_value, 1)
        self.pid_graph_j22.addData(value.process_value_dot, 2)
        self.pid_graph_j22.addData(value.error, 3)
        self.pid_graph_j22.addData(value.command*self.pid_output_scale, 4)

    def j23_pid_cb(self, value):
        self.pid_graph_j23.addData(value.set_point, 0)
        self.pid_graph_j23.addData(value.process_value, 1)
        self.pid_graph_j23.addData(value.process_value_dot, 2)
        self.pid_graph_j23.addData(value.error, 3)
        self.pid_graph_j23.addData(value.command*self.pid_output_scale, 4)


    def pid_graphs_cb(self, value):
        self.pid_graph.addData(value.set_point, 0)
        self.pid_graph.addData(value.process_value, 1)
        self.pid_graph.addData(value.process_value_dot, 2)
        self.pid_graph.addData(value.error, 3)
        self.pid_graph.addData(value.command, 4)

        self.pid_clipped_graph.addData(value.set_point, 0)
        self.pid_clipped_graph.addData(value.process_value, 1)
        self.pid_clipped_graph.addData(value.process_value_dot, 2)
        self.pid_clipped_graph.addData(value.error, 3)
        self.pid_clipped_graph.addData(value.command, 4)


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

    def init_widget_children(self):
        #Overview page
        self.pid_clipped_layout = self._widget.findChild(QVBoxLayout, "pid_clipped_layout")
        self.pid_layout         = self._widget.findChild(QVBoxLayout, "pid_layout")
        self.pos_vel_eff_layout = self._widget.findChild(QVBoxLayout, "pos_vel_eff_layout")
        self.palm_extras_layout = self._widget.findChild(QVBoxLayout, "palm_extras_layout")
        self.biotac_overview_layout = self._widget.findChild(QVBoxLayout, "biotac_overview_layout")
        self.effort_layout = self._widget.findChild(QVBoxLayout, "effort_layout")

        #Biotac page
        self.biotac_0_layout = self._widget.findChild(QVBoxLayout, "biotac_0_layout")
        self.biotac_1_layout = self._widget.findChild(QVBoxLayout, "biotac_1_layout")
        self.biotac_2_layout = self._widget.findChild(QVBoxLayout, "biotac_2_layout")
        self.biotac_3_layout = self._widget.findChild(QVBoxLayout, "biotac_3_layout")
        self.biotac_4_layout = self._widget.findChild(QVBoxLayout, "biotac_4_layout")
        self.biotacs_tac_layout = self._widget.findChild(QVBoxLayout, "biotacs_tac_layout")
        self.biotacs_pdc_layout = self._widget.findChild(QVBoxLayout, "biotacs_pdc_layout")
        self.biotacs_pac_layout = self._widget.findChild(QVBoxLayout, "biotacs_pac_layout")

        #All joints page
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

        self.j0_pid_layout = self._widget.findChild(QVBoxLayout, "j0_layout_2")
        self.j1_pid_layout = self._widget.findChild(QVBoxLayout, "j1_layout_2")
        self.j2_pid_layout = self._widget.findChild(QVBoxLayout, "j2_layout_2")
        self.j3_pid_layout = self._widget.findChild(QVBoxLayout, "j3_layout_2")
        self.j4_pid_layout = self._widget.findChild(QVBoxLayout, "j4_layout_2")
        self.j5_pid_layout = self._widget.findChild(QVBoxLayout, "j5_layout_2")
        self.j6_pid_layout = self._widget.findChild(QVBoxLayout, "j6_layout_2")
        self.j7_pid_layout = self._widget.findChild(QVBoxLayout, "j7_layout_2")
        self.j8_pid_layout = self._widget.findChild(QVBoxLayout, "j8_layout_2")
        self.j9_pid_layout = self._widget.findChild(QVBoxLayout, "j9_layout_2")
        self.j10_pid_layout = self._widget.findChild(QVBoxLayout, "j10_layout_2")
        self.j11_pid_layout = self._widget.findChild(QVBoxLayout, "j11_layout_2")
        self.j12_pid_layout = self._widget.findChild(QVBoxLayout, "j12_layout_2")
        self.j13_pid_layout = self._widget.findChild(QVBoxLayout, "j13_layout_2")
        self.j14_pid_layout = self._widget.findChild(QVBoxLayout, "j14_layout_2")
        self.j15_pid_layout = self._widget.findChild(QVBoxLayout, "j15_layout_2")
        self.j16_pid_layout = self._widget.findChild(QVBoxLayout, "j16_layout_2")
        self.j17_pid_layout = self._widget.findChild(QVBoxLayout, "j17_layout_2")
        self.j18_pid_layout = self._widget.findChild(QVBoxLayout, "j18_layout_2")
        self.j19_pid_layout = self._widget.findChild(QVBoxLayout, "j19_layout_2")
        self.j20_pid_layout = self._widget.findChild(QVBoxLayout, "j20_layout_2")
        self.j21_pid_layout = self._widget.findChild(QVBoxLayout, "j21_layout_2")
        self.j22_pid_layout = self._widget.findChild(QVBoxLayout, "j22_layout_2")
        self.j23_pid_layout = self._widget.findChild(QVBoxLayout, "j23_layout_2")


    def create_menu_bar(self):
        self._widget.myQMenuBar = QMenuBar(self._widget)
        fileMenu = self._widget.myQMenuBar.addMenu('&File')


class CustomFigCanvas(FigureCanvas, TimedAnimation):
###https://stackoverflow.com/questions/36665850/matplotlib-animation-inside-your-own-pyqt4-gui
    def __init__(self, num_lines, colour = [], ymin = -1, ymax = 1, legends = [], rows=1):
        self.num_lines = num_lines
        self.rows = rows
        self.addedDataArray = []
        n = 0
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
        while i < self.num_lines:
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
           ncol=self.num_lines/self.rows, mode="expand", borderaxespad=0., prop={'size': 7})

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
