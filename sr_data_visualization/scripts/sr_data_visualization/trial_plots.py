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

import random
import numpy as np
import rospy

from qtpy.QtWidgets import QFrame
from qtpy.QtGui import QPen, QBrush
from qtpy.QtCore import QSize, Qt
from qwt import (
    QwtPlot,
    QwtPlotMarker,
    QwtSymbol,
    QwtLegend,
    QwtPlotCurve,
    QwtAbstractScaleDraw,
)

from sensor_msgs.msg import JointState
from sr_utilities.hand_finder import HandFinder

class DataPlot(QwtPlot):
    def __init__(self, unattended=False):
        QwtPlot.__init__(self)

        self._hand_finder = HandFinder()
        self._hand_parameters = self._hand_finder.get_hand_parameters()
        self._joint_prefix = next(iter(list(self._hand_parameters.joint_prefix.values())))

        self._joint_name =  self._joint_prefix + "FFJ1"
        
        self.setCanvasBackground(Qt.white)
        self.axisAutoScale(QwtPlot.xBottom)
        self.axisAutoScale(QwtPlot.yLeft)
        # self.alignScales()

        # Initialize data
        self.x = np.arange(0.0, 100.1, 0.5)
        self.position_data = np.zeros(len(self.x), float)
        self.effort_data = np.zeros(len(self.x), float)
        self.velocity_data = np.zeros(len(self.x), float)

        self.setTitle("Joint States")
        self.insertLegend(QwtLegend(), QwtPlot.BottomLegend)

        self.position_plot = QwtPlotCurve("Position")
        self.position_plot.attach(self)
        self.effort_plot = QwtPlotCurve("Effort")
        self.effort_plot.attach(self)
        self.velocity_plot = QwtPlotCurve("Effort")
        self.velocity_plot.attach(self)

        # self.position_plot.setSymbol(
        #     QwtSymbol(QwtSymbol.Ellipse, QBrush(), QPen(Qt.yellow), QSize(7, 7))
        # )

        self.position_plot.setPen(QPen(Qt.red))
        self.effort_plot.setPen(QPen(Qt.blue))
        self.velocity_plot.setPen(QPen(Qt.green))

        mY = QwtPlotMarker()
        mY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        mY.setLineStyle(QwtPlotMarker.HLine)
        mY.setYValue(0.0)
        mY.attach(self)

        # self.setAxisTitle(QwtPlot.xBottom, "Time (seconds)")
        # self.setAxisTitle(QwtPlot.yLeft, "Values")

        self.ready_for_new = True
        self.joint_state_data = {
            'Position': 0.0,
            'Effort': 0.0,
            'Velocity': 0.0
        }

        self._joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self._joint_state_cb, queue_size=1)

        self.startTimer(10 if unattended else 50)
        self.phase = 0.0

    def _joint_state_cb(self, joint_state):
        if self.ready_for_new:
            for name, position, velocity, effort in zip(joint_state.name, joint_state.position,
                                                            joint_state.velocity, joint_state.effort):
                if name == self._joint_name:
                    self.joint_state_data['Position'] = position
                    self.joint_state_data['Effort'] = effort
                    self.joint_state_data['Velocity'] = velocity
        # rospy.logerr(str(self.joint_state_data))

    def alignScales(self):
        self.canvas().setFrameStyle(QFrame.Box | QFrame.Plain)
        self.canvas().setLineWidth(1)
        for axis_id in QwtPlot.AXES:
            scaleWidget = self.axisWidget(axis_id)
            if scaleWidget:
                scaleWidget.setMargin(0)
            scaleDraw = self.axisScaleDraw(axis_id)
            if scaleDraw:
                scaleDraw.enableComponent(QwtAbstractScaleDraw.Backbone, False)

    def new_data(self, data_type, data):
        # data moves from left to right:
        # shift data array right and assign new value data[0]
        data = np.concatenate((data[:1], data[:-1]))
        data[0] = data[data_type]
        return data

    def timerEvent(self, e):
        self.ready_for_new = False
        # self.position_plot.setData(self.x, self.new_data('Position', self.position_data))
        # self.effort_plot.setData(self.x, self.new_data('Effort', self.effort_data))
        # self.velocity_plot.setData(self.x,self.new_data('Velocity', self.velocity_data))
        # self.replot()
        # self.ready_for_new = True


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
        self.ready_for_new = True

        # # y moves from left to right:
        # # shift y array right and assign new value y[0]
        # self.y = np.concatenate((self.y[:1], self.y[:-1]))
        # self.y[0] = np.sin(self.phase) * (-1.0 + 2.0 * random.random())

        # # z moves from right to left:
        # # Shift z array left and assign new value to z[n-1].
        # self.z = np.concatenate((self.z[1:], self.z[:1]))
        # self.z[-1] = 0.8 - (2.0 * self.phase / np.pi) + 0.4 * random.random()

        # self.curveR.setData(self.x, self.y)
        # self.curveL.setData(self.x, self.z)

        # self.replot()
        # self.phase += np.pi * 0.02


if __name__ == "__main__":
    from qwt import tests
    rospy.init_node('trial_plots', anonymous=True)
    app = tests.test_widget(DataPlot, size=(500, 300))