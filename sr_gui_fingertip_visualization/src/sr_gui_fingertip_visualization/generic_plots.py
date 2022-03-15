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
        self._plot.setData(np.linspace(0, 10, len(data)), data)
    
    def get_plot(self):
        return self._plot


class GenericDataPlot(QwtPlot):
    _GRAPH_MINW = 50
    _GRAPH_MINH = 30

    def __init__(self, data, colors):
        super().__init__()
        self._colors = colors
        self.setCanvasBackground(Qt.white)
        #self.setMinimumSize(self._GRAPH_MINW, self._GRAPH_MINH)
        #rospy.logwarn(self.size) 
        #self.axisScaleDraw(QwtPlot.xBottom).enableComponent(QwtScaleDraw.Labels, False)
        #self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, False)
        self._traces = dict()
        self.generate_plots(data)

    def generate_plots(self, data):
        self._data_fields = list(data.keys())
        for data_field in self._data_fields:
            self._traces[data_field] = Trace(data_field, self._colors[data_field]['plot_color'])

    def show_traces(self, selected_traces):
        if selected_traces:
            for data_field in self._data_fields:
                self._traces[data_field].get_plot().detach()
                if data_field in selected_traces:                        
                    #self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, True)
                    #self.axisAutoScale(QwtPlot.yLeft)
                    self._traces[data_field].get_plot().attach(self)   

    def update_plot(self, data):
        for data_field in list(data.keys()):
            self._traces[data_field].update_trace_data(data[data_field])



