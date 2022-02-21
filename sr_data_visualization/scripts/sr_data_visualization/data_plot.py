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
    QwtScaleDraw
)


class Trace():
    def __init__(self, trace_name, trace_colour, x): #, min_val, max_val
        self.name = trace_name
        self.plot = QwtPlotCurve(trace_name)
        self.plot.setPen(trace_colour)
        self.data = np.zeros(len(x), float)
        self.cb_data = 0.0
        # self.max_val = max_val
        # self.min_val = min_val


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

        self.axisScaleDraw(QwtPlot.xBottom).enableComponent(QwtScaleDraw.Labels, False)
        self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, False)
        # self.setAxisScale(QwtPlot.yLeft, -3.0, 3.0)

        # Initialize data
        self.x = np.arange(0.0, 100.1, 0.5)

        # self.insertLegend(QwtLegend(), QwtPlot.TopLegend)
        # self.setAxisTitle(QwtPlot.xBottom, "Time (seconds)")
        # self.setAxisScale

        # self.setAxisScale(QwtPlot.yLeft, 0, 1.8,0.5 )
        # self.setAxisScale(QwtPlot.yLeft, 0, 1000, 0.5 )

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

    def show_trace(self, trace_name):
        for trace in self.traces:
            if trace_name == trace.name:
                self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, True)
                # self.plot().setAxisScale(QwtPlot.yLeft, trace.min_val, trace.max_val)
                trace.plot.attach(self)
            elif trace_name == "All":
                trace.plot.attach(self)
                self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, False)
            else:
                trace.plot.detach()


class JointStatesDataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Position", QPen(Qt.red), self.x), #, -3.14, 3.14
                       Trace("Effort", QPen(Qt.blue), self.x), # , -3.14, 3.14
                       Trace("Velocity", QPen(Qt.green), self.x)] # , -600, 600

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


class MotorStats1DataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)
        self.joint_name = joint_name

    def create_traces(self):
        self.traces = [Trace("Strain Gauge Right", QPen(Qt.red), self.x),
                       Trace("Strain Gauge Left", QPen(Qt.blue), self.x),
                       Trace("Measured PWM", QPen(Qt.green), self.x),
                       Trace("Measured Current", QPen(Qt.yellow), self.x),
                       Trace("Measured Voltage", QPen(Qt.magenta), self.x)]

    def _callback(self, data):
        for message in data.status:
            parts = message.name.split('/')
            if len(parts) == 4:
                parts = parts[3].split(' ')
                if len(parts) == 3 and parts[1] == 'SRDMotor':
                    joint = parts[0] + '_' + parts[2]
                    if joint in self.joint_name:
                        for item in message.values:
                            if item.key == self.traces[0].name:
                                self.traces[0].cb_data = item.value
                            elif item.key == self.traces[1].name:
                                self.traces[1].cb_data = item.value
                            elif item.key == self.traces[2].name:
                                self.traces[2].cb_data = item.value
                            elif item.key == self.traces[3].name:
                                self.traces[3].cb_data = item.value
                            elif item.key == self.traces[4].name:
                                self.traces[4].cb_data = item.value


class MotorStats2DataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)
        self.joint_name = joint_name

    def create_traces(self):
        self.traces = [Trace("Measured Effort", QPen(Qt.red), self.x),
                       Trace("Temperature", QPen(Qt.blue), self.x),
                       Trace("Unfiltered position", QPen(Qt.green), self.x),
                       Trace("Unfiltered force", QPen(Qt.yellow), self.x),
                       Trace("Last Commanded Effort", QPen(Qt.magenta), self.x),
                       Trace("Encoder Position", QPen(Qt.cyan), self.x)]

    def _callback(self, data):
        for message in data.status:
            parts = message.name.split('/')
            if len(parts) == 4:
                parts = parts[3].split(' ')
                if len(parts) == 3 and parts[1] == 'SRDMotor':
                    joint = parts[0] + '_' + parts[2]
                    if joint in self.joint_name:
                        for item in message.values:
                            if item.key == self.traces[0].name:
                                self.traces[0].cb_data = item.value
                            elif item.key == self.traces[1].name:
                                self.traces[1].cb_data = item.value
                            elif item.key == self.traces[2].name:
                                self.traces[2].cb_data = item.value
                            elif item.key == self.traces[3].name:
                                self.traces[3].cb_data = item.value
                            elif item.key == self.traces[4].name:
                                self.traces[4].cb_data = item.value
                            elif item.key == self.traces[5].name:
                                self.traces[5].cb_data = item.value


class PalmExtrasAcellDataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Accel X", QPen(Qt.red), self.x),
                       Trace("Accel Y", QPen(Qt.blue), self.x),
                       Trace("Accel Z", QPen(Qt.green), self.x)]

    def _callback(self, data):
        self.traces[0].cb_data = data.data[0]
        self.traces[1].cb_data = data.data[1]
        self.traces[2].cb_data = data.data[2]


class PalmExtrasGyroDataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Gyro X", QPen(Qt.red), self.x),
                       Trace("Gyro Y", QPen(Qt.blue), self.x),
                       Trace("Gyro Z", QPen(Qt.green), self.x)]

    def _callback(self, data):
        self.traces[0].cb_data = data.data[3]
        self.traces[1].cb_data = data.data[4]
        self.traces[2].cb_data = data.data[5]


class PalmExtrasADCDataPlot(GenericDataPlot):
    """
        Creates the QwtPlot of the data
    """
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("ADC0", QPen(Qt.red), self.x),
                       Trace("ADC1", QPen(Qt.blue), self.x),
                       Trace("ADC2", QPen(Qt.green), self.x),
                       Trace("ADC3", QPen(Qt.yellow), self.x)]

    def _callback(self, data):
        self.traces[0].cb_data = data.data[6]
        self.traces[1].cb_data = data.data[7]
        self.traces[2].cb_data = data.data[8]
        self.traces[3].cb_data = data.data[9]
