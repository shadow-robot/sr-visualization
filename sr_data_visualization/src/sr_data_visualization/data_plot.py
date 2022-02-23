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

from python_qt_binding.QtGui import QPen
from python_qt_binding.QtCore import Qt, QTimer

from qwt import (
    QwtPlot,
    QwtPlotCurve,
    QwtScaleDraw
)


class Trace():
    def __init__(self, trace_name, qt_colour, x_data):
        self.name = trace_name
        self.plot = QwtPlotCurve(trace_name)
        self.plot.setPen(QPen(qt_colour))
        self.data = np.zeros(x_data.shape)
        self.latest_value = 0.0


class GenericDataPlot(QwtPlot):
    def __init__(self, joint_name, topic_name, topic_type, start_plotting=False):
        super().__init__()

        self.joint_name = joint_name
        self._topic_name = topic_name
        self._topic_type = topic_type

        self.setCanvasBackground(Qt.white)
        self.setMinimumSize(150, 50)

        self.axisScaleDraw(QwtPlot.xBottom).enableComponent(QwtScaleDraw.Labels, False)
        self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, False)

        # Initialize data
        self.x_data = np.arange(0.0, 100.1, 0.5)

        self.create_traces()
        for trace in self.traces:
            trace.plot.attach(self)

        self._subscriber = rospy.Subscriber(self._topic_name, self._topic_type,
                                            self.callback, queue_size=1)

        self.initialize_and_start_timer()

        if not start_plotting:
            self.plot_data(False)

    def create_traces(self):
        raise NotImplementedError("The function create_traces must be implemented")

    def callback(self, data):
        raise NotImplementedError("The function callback must be implemented")

    def initialize_and_start_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start()

    def timerEvent(self):
        # Data moves from left to right:
        # Shift data array right and assign new value data[0]
        for trace in self.traces:
            trace.data = np.concatenate((trace.data[:1], trace.data[:-1]))
            trace.data[0] = trace.latest_value
            trace.plot.setData(self.x_data, trace.data)

        self.replot()

    def plot_data(self, plot):
        if plot:
            self._subscriber = rospy.Subscriber(self._topic_name, self._topic_type,
                                                self.callback, queue_size=1)
            self.timer.start()
        else:
            self._subscriber.unregister()
            self.timer.stop()

    def show_trace(self, trace_name):
        for trace in self.traces:
            if trace_name == trace.name:
                self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, True)
                self.axisAutoScale(QwtPlot.yLeft)
                trace.plot.attach(self)
            elif trace_name == "All":
                self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, False)
                trace.plot.attach(self)
            else:
                trace.plot.detach()


class JointStatesDataPlot(GenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type, start_plotting=True)

    def create_traces(self):
        self.traces = [Trace("Position", Qt.red, self.x_data),
                       Trace("Effort", Qt.blue, self.x_data),
                       Trace("Velocity", Qt.green, self.x_data)]

    def callback(self, data):
        for name, position, velocity, effort in zip(data.name, data.position,
                                                    data.velocity, data.effort):
            if name == self.joint_name:
                self.traces[0].latest_value = position
                self.traces[1].latest_value = effort
                self.traces[2].latest_value = velocity


class ControlLoopsDataPlot(GenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Set Point", Qt.red, self.x_data),
                       Trace("Input", Qt.blue, self.x_data),
                       Trace("dInput/dt", Qt.green, self.x_data),
                       Trace("Error", Qt.yellow, self.x_data),
                       Trace("Output", Qt.magenta, self.x_data)]

    def callback(self, data):
        self.traces[0].latest_value = data.set_point
        self.traces[1].latest_value = data.process_value
        self.traces[2].latest_value = data.process_value_dot
        self.traces[3].latest_value = data.error
        self.traces[4].latest_value = data.command


class MotorStatsGenericDataPlot(GenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def callback(self, data):
        for message in data.status:
            # Splits the name into parts e.g.
            # name: "/Right Shadow Hand/Wrist/rh SRDMotor WRJ2"
            # parts = ['', 'Right Shadow Hand', 'Wrist', 'rh SRDMotor WRJ2']
            parts = message.name.split('/')
            if len(parts) == 4:
                # Splits the 4th part into words & decides if it is a Motor
                parts = parts[3].split(' ')
                # Find SRDMotor part andthen use this to locate
                # the values for the specific joint
                if len(parts) == 3 and parts[1] == 'SRDMotor':
                    joint = parts[0] + '_' + parts[2]
                    if joint in self.joint_name:
                        for item in message.values:
                            for trace in range(len(self.traces)):
                                if item.key == self.traces[trace].name:
                                    print(str(item.key) + ": " + str(item.value))
                                    self.traces[trace].latest_value = item.value


class MotorStats1DataPlot(MotorStatsGenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Strain Gauge Right", Qt.red, self.x_data),
                       Trace("Strain Gauge Left", Qt.blue, self.x_data),
                       Trace("Measured PWM", Qt.green, self.x_data),
                       Trace("Measured Current", Qt.yellow, self.x_data),
                       Trace("Measured Voltage", Qt.magenta, self.x_data)]


class MotorStats2DataPlot(MotorStatsGenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Measured Effort", Qt.red, self.x_data),
                       Trace("Temperature", Qt.blue, self.x_data),
                       Trace("Unfiltered position", Qt.green, self.x_data),
                       Trace("Unfiltered force", Qt.yellow, self.x_data),
                       Trace("Last Commanded Effort", Qt.magenta, self.x_data),
                       Trace("Encoder Position", Qt.cyan, self.x_data)]


class PalmExtrasAcellDataPlot(GenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Accel X", Qt.red, self.x_data),
                       Trace("Accel Y", Qt.blue, self.x_data),
                       Trace("Accel Z", Qt.green, self.x_data)]

    def callback(self, data):
        self.traces[0].latest_value = data.data[0]
        self.traces[1].latest_value = data.data[1]
        self.traces[2].latest_value = data.data[2]


class PalmExtrasGyroDataPlot(GenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("Gyro X", Qt.yellow, self.x_data),
                       Trace("Gyro Y", Qt.magenta, self.x_data),
                       Trace("Gyro Z", Qt.cyan, self.x_data)]

    def callback(self, data):
        self.traces[0].latest_value = data.data[3]
        self.traces[1].latest_value = data.data[4]
        self.traces[2].latest_value = data.data[5]


class PalmExtrasADCDataPlot(GenericDataPlot):
    def __init__(self, joint_name, topic_name, topic_type):
        super().__init__(joint_name, topic_name, topic_type)

    def create_traces(self):
        self.traces = [Trace("ADC0", Qt.red, self.x_data),
                       Trace("ADC1", Qt.blue, self.x_data),
                       Trace("ADC2", Qt.green, self.x_data),
                       Trace("ADC3", Qt.yellow, self.x_data)]

    def callback(self, data):
        self.traces[0].latest_value = data.data[6]
        self.traces[1].latest_value = data.data[7]
        self.traces[2].latest_value = data.data[8]
        self.traces[3].latest_value = data.data[9]
