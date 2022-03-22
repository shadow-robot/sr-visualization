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

from python_qt_binding.QtGui import QPen, QColor
from python_qt_binding.QtCore import Qt

from qwt import (
    QwtPlot,
    QwtPlotCurve,
    QwtScaleDraw
)


class Trace():
    def __init__(self, name, color):
        self.name = name
        self._plot = QwtPlotCurve(name)
        self._plot.setPen(QPen(color))
        self._data = None

    def update_trace_data(self, data):
        self._data = data
        self._plot.setData(np.linspace(0, 100, len(data)), data)

    def get_plot(self):
        return self._plot


class GenericDataPlot(QwtPlot):
    _GRAPH_MINW = 300
    _GRAPH_MINH = 200

    def __init__(self, data, colors):
        super().__init__()
        self._colors = colors
        self.setCanvasBackground(Qt.white)
        self.setMaximumSize(self._GRAPH_MINW, self._GRAPH_MINH)
        self.axisScaleDraw(QwtPlot.xBottom).enableComponent(QwtScaleDraw.Labels, False)
        self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, False)

        self._traces = dict()
        self.generate_plots(data)

    def generate_plots(self, data):
        self._data_fields = list(data.keys())
        for i, data_field in enumerate(self._data_fields):
            self._traces[data_field] = Trace(data_field, QColor(self._colors[i]))

    def show_trace(self, data_field, show=True):
        self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, True)
        self.axisAutoScale(QwtPlot.yLeft)
        if show:
            self._traces[data_field].get_plot().attach(self)
            return
        self._traces[data_field].get_plot().detach()

    def update_plot(self, data):
        for data_field in list(data.keys()):
            if self._traces[data_field].get_plot().plot():
                self._traces[data_field].update_trace_data(data[data_field])

    def get_data_fields(self):
        return self._data_fields
