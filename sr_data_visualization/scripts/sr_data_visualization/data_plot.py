#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
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

from __future__ import absolute_import

import numpy as np
import rospy

from qtpy.QtGui import QPen
from qtpy.QtCore import Qt, QTimer
from qwt import (
    QwtPlot,
    QwtLegend,
    QwtPlotCurve,
    QwtAbstractScaleDraw,
)

from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState


class JointStatesDataPlot(QwtPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name):
        QwtPlot.__init__(self)

        self._joint_name = joint_name

        self.setCanvasBackground(Qt.white)
        self.setMinimumSize(250, 100)

        # Initialize data
        self.x = np.arange(0.0, 100.1, 0.5)
        self.position_data = np.zeros(len(self.x), float)
        self.effort_data = np.zeros(len(self.x), float)
        self.velocity_data = np.zeros(len(self.x), float)

        # self.setTitle(self._joint_name)
        self.insertLegend(QwtLegend(), QwtPlot.TopLegend)


        # Create plots
        self.position_plot = QwtPlotCurve("Position")
        self.position_plot.attach(self)
        self.effort_plot = QwtPlotCurve("Effort")
        self.effort_plot.attach(self)
        self.velocity_plot = QwtPlotCurve("Velocity")
        self.velocity_plot.attach(self)

        self.position_plot.setPen(QPen(Qt.red))
        self.effort_plot.setPen(QPen(Qt.blue))
        self.velocity_plot.setPen(QPen(Qt.green))

        self.setAxisTitle(QwtPlot.xBottom, "Time (seconds)")

        self.ready_for_new = True
        self.joint_state_data = {
            'Position': 0.0,
            'Effort': 0.0,
            'Velocity': 0.0
        }

        self._joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self._joint_state_cb, queue_size=1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start()

    def _joint_state_cb(self, joint_state):
        for name, position, velocity, effort in zip(joint_state.name, joint_state.position,
                                                    joint_state.velocity, joint_state.effort):
            if name == self._joint_name:
                self.joint_state_data['Position'] = position
                self.joint_state_data['Effort'] = effort
                self.joint_state_data['Velocity'] = velocity

    def timerEvent(self):
        # Data moves from left to right:
        # Shift data array right and assign new value data[0]
        self.position_data = np.concatenate((self.position_data[:1], self.position_data[:-1]))
        self.position_data[0] = self.joint_state_data['Position']

        self.effort_data = np.concatenate((self.effort_data[:1], self.effort_data[:-1]))
        self.effort_data[0] = self.joint_state_data['Effort']

        self.velocity_data = np.concatenate((self.velocity_data[:1], self.velocity_data[:-1]))
        self.velocity_data[0] = self.joint_state_data['Velocity']

        self.position_plot.setData(self.x, self.position_data)
        self.effort_plot.setData(self.x, self.effort_data)
        self.velocity_plot.setData(self.x, self.velocity_data)
        self.replot()

    def plot_data(self, plot):
        if plot:
            self._joint_states_subscriber = rospy.Subscriber('joint_states',
                                                             JointState,
                                                             self._joint_state_cb,
                                                             queue_size=1)
            self.timer.start()
        else:
            self.timer.stop()
            self._joint_states_subscriber.unregister()

    def turn_off_trace(self, trace_name):
        if trace_name == "position":
            self.position_plot.attach(self)
            self.effort_plot.detach()
            self.velocity_plot.detach()
        if trace_name == "velocity":
            self.velocity_plot.attach(self)
            self.effort_plot.detach()
            self.position_plot.detach()
        if trace_name == "effort":
            self.effort_plot.attach(self)
            self.velocity_plot.detach()
            self.position_plot.detach()
        if trace_name == "all":
            self.position_plot.attach(self)
            self.velocity_plot.attach(self)
            self.effort_plot.attach(self)


class ControlLoopsDataPlot(QwtPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, unattended=False):
        QwtPlot.__init__(self)

        self._joint_name = joint_name
        self.unattended = unattended

        self.setCanvasBackground(Qt.white)
        self.setMinimumSize(250, 100)

        # Initialize data
        self.x = np.arange(0.0, 100.1, 0.5)
        self.setpoint_data = np.zeros(len(self.x), float)
        self.input_data = np.zeros(len(self.x), float)
        self.dinputdt_data = np.zeros(len(self.x), float)
        self.error_data = np.zeros(len(self.x), float)
        self.output_data = np.zeros(len(self.x), float)

        # self.setTitle(self._joint_name)
        self.insertLegend(QwtLegend(), QwtPlot.TopLegend)

        # Create plots
        self.setpoint_plot = QwtPlotCurve("Set Point")
        self.setpoint_plot.attach(self)
        self.input_plot = QwtPlotCurve("Input")
        self.input_plot.attach(self)
        self.dinputdt_plot = QwtPlotCurve("dInput/dt")
        self.dinputdt_plot.attach(self)
        self.error_plot = QwtPlotCurve("Error")
        self.error_plot.attach(self)
        self.output_plot = QwtPlotCurve("Output")
        self.output_plot.attach(self)

        self.setpoint_plot.setPen(QPen(Qt.red))
        self.input_plot.setPen(QPen(Qt.blue))
        self.dinputdt_plot.setPen(QPen(Qt.green))
        self.error_plot.setPen(QPen(Qt.yellow))
        self.output_plot.setPen(QPen(Qt.magenta))

        # self.setAxisTitle(QwtPlot.xBottom, "Time (seconds)")

        self.joint_state_data = {
            'Set Point': 0.0,
            'Input': 0.0,
            'dInput/dt': 0.0,
            'Error': 0.0,
            'Output': 0.0
        }

        self._position_controller_subscriber = rospy.Subscriber('/sh_' + self._joint_name.lower() + '_position_controller/state',
                                                                JointControllerState,
                                                                self._position_controller_cb,
                                                                queue_size=1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timerEvent)
        # self.timer.start()
        self.timer.start(10 if self.unattended else 50)


    def _position_controller_cb(self, data):
        self.joint_state_data['Set Point'] = data.set_point
        self.joint_state_data['Input'] = data.process_value
        self.joint_state_data['dInput/dt'] = data.process_value_dot
        self.joint_state_data['Error'] = data.error
        self.joint_state_data['Output'] = data.command

    def timerEvent(self):
        # Data moves from left to right:
        # Shift data array right and assign new value data[0]
        self.setpoint_data = np.concatenate((self.setpoint_data[:1], self.setpoint_data[:-1]))
        self.setpoint_data[0] = self.joint_state_data['Set Point']

        self.input_data = np.concatenate((self.input_data[:1], self.input_data[:-1]))
        self.input_data[0] = self.joint_state_data['Input']

        self.dinputdt_data = np.concatenate((self.dinputdt_data[:1], self.dinputdt_data[:-1]))
        self.dinputdt_data[0] = self.joint_state_data['dInput/dt']

        self.error_data = np.concatenate((self.error_data[:1], self.error_data[:-1]))
        self.error_data[0] = self.joint_state_data['Error']

        self.output_data = np.concatenate((self.output_data[:1], self.output_data[:-1]))
        self.output_data[0] = self.joint_state_data['Output']



        self.setpoint_plot.setData(self.x, self.setpoint_data)
        self.input_plot.setData(self.x, self.input_data)
        self.dinputdt_plot.setData(self.x, self.dinputdt_data)
        self.error_plot.setData(self.x, self.error_data)
        self.output_plot.setData(self.x, self.output_data)
        self.replot()

    def plot_data(self, plot):
        if plot:
            self._position_controller_subscriber = rospy.Subscriber('/sh_' + self._joint_name.lower() + '_position_controller/state',
                                                                    JointControllerState,
                                                                    self._position_controller_cb,
                                                                    queue_size=1)
            self.timer.start()
        else:
            self.timer.stop()
            self._joint_states_subscriber.unregister()

    def turn_off_trace(self, trace_name):
        if trace_name == "setpoint":
            self.setpoint_plot.attach(self)
            self.input_plot.detach()
            self.dinputdt_plot.detach()
            self.error_plot.detach()
            self.output_plot.detach()
        elif trace_name == "input":
            self.input_plot.attach(self)
            self.dinputdt_plot.detach()
            self.setpoint_plot.detach()
            self.error_plot.detach()
            self.output_plot.detach()
        elif trace_name == "dinputdt":
            self.dinputdt_plot.attach(self)
            self.input_plot.detach()
            self.setpoint_plot.detach()
            self.error_plot.detach()
            self.output_plot.detach()
        elif trace_name == "error":
            self.error_plot.attach(self)
            self.input_plot.detach()
            self.setpoint_plot.detach()
            self.dinputdt_plot.detach()
            self.output_plot.detach()
        elif trace_name == "output":
            self.output_plot.attach(self)
            self.input_plot.detach()
            self.setpoint_plot.detach()
            self.error_plot.detach()
            self.dinputdt_plot.detach()
        elif trace_name == "all":
            self.setpoint_plot.attach(self)
            self.input_plot.attach(self)
            self.dinputdt_plot.attach(self)
            self.error_plot.attach(self)
            self.output_plot.attach(self)
