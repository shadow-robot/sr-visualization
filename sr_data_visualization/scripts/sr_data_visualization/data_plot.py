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

class Trace():
    def __init__(self, trace_name, trace_colour, x):
        self.name = trace_name
        self.plot = QwtPlotCurve(trace_name)
        self.plot.setPen(trace_colour)
        self.data = np.zeros(len(x), float)
        self.cb_data = 0.0


class GenericDataPlot(QwtPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        QwtPlot.__init__(self)

        self._joint_name = joint_name
        self.topic_name = topic_name
        self.topic_type = topic_type

        self.setCanvasBackground(Qt.white)
        self.setMinimumSize(250, 100)

        # Initialize data
        self.x = np.arange(0.0, 100.1, 0.5)

        self.insertLegend(QwtLegend(), QwtPlot.TopLegend)
        self.setAxisTitle(QwtPlot.xBottom, "Time (seconds)")

        self.create_traces()
        for trace in self.traces:
            trace.plot.attach(self)

        self._subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback, queue_size=1)

        self.start_timer()

    def create_traces(self):
        pass


    def _callback(self, data):
        pass

    def start_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start()

    def timerEvent(self):
        # Data moves from left to right:
        # Shift data array right and assign new value data[0]
        for trace in self.traces:
            trace.data = np.concatenate((trace.data[:1], trace.data[:-1]))
            trace.data[0] = trace.cb_data
            trace.plot.setData(self.x, trace.data)

        self.replot()

    def plot_data(self, plot):
        if plot:
            self._subscriber = rospy.Subscriber(self.topic_name, self.topic_type,
                                                self._callback, queue_size=1)
            self.timer.start()
        else:
            self.timer.stop()
            self._subscriber.unregister()

    def turn_off_trace(self, trace_name):
        for trace in self.traces:
            if trace_name == "All" or trace_name == trace.name:
                trace.plot.attach(self)
            else:
                trace.plot.detach()


class JointStatesDataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Position", QPen(Qt.red), self.x),
                       Trace("Effort", QPen(Qt.blue), self.x),
                       Trace("Velocity", QPen(Qt.green), self.x)]

    def _callback(self, data):
        for name, position, velocity, effort in zip(data.name, data.position,
                                                    data.velocity, data.effort):
            if name == self._joint_name:
                self.traces[0].cb_data = position
                self.traces[1].cb_data = effort
                self.traces[2].cb_data = velocity


class ControlLoopsDataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Set Point", QPen(Qt.red), self.x),
                       Trace("Input", QPen(Qt.blue), self.x),
                       Trace("dInput/dt", QPen(Qt.green), self.x),
                       Trace("Error", QPen(Qt.yellow), self.x),
                       Trace("Output", QPen(Qt.magenta), self.x)]

    def _callback(self, data):
        self.traces[0].cb_data = data.set_point
        self.traces[1].cb_data = data.process_value
        self.traces[2].cb_data = data.process_value_dot
        self.traces[3].cb_data = data.error
        self.traces[4].cb_data = data.command
