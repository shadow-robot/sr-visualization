#!/usr/bin/env python3

# Copyright 2022-2023 Shadow Robot Company Ltd.
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

from typing import Dict, List, Tuple
import numpy as np
from python_qt_binding.QtGui import QPen, QColor
from python_qt_binding.QtCore import Qt
from qwt import (
    QwtPlot,
    QwtPlotCurve,
    QwtScaleDraw
)


class Trace():
    def __init__(self, name, color) -> None:
        '''
            Initialize the trace.
            @param name: The name of the trace
            @param color: The color of the trace
        '''
        self.name = name
        self._plot = QwtPlotCurve(name)
        self._plot.setPen(QPen(color))
        self._x_data = np.arange(0.0, 100.1, 0.5)
        self._y_data = np.zeros(self._x_data.shape)

    def update_trace_data(self, data: np.ndarray) -> None:
        '''
            Update the data of the trace. If the new data is shorter than the old data, the old data is shifted to the right
            and the new data is added to the left. If the new data is longer than the old data, the old data is replaced
            with the new data.
            @param data: The new data to be added to the trace
        '''
        if len(data) < len(self._y_data):
            self._y_data = np.concatenate((data, self._y_data[:-len(data)]))
        else:
            self._y_data = data[-len(self._y_data):]
        self._plot.setData(self._x_data, self._y_data)

    def get_min_max_for_axis(self) -> Tuple[float, float]:
        '''
            Get the minimum and maximum values for the y axis of the plot. If the minimum and maximum values are the same,
            the minimum value is decreased by 0.1 and the maximum value is increased by 0.1.
            @return: The minimum and maximum values for the y axis of the plot
        '''
        if np.min(self._y_data) == np.max(self._y_data):
            return np.min(self._y_data) - 0.1, np.max(self._y_data) + 0.1
        return np.min(self._y_data), np.max(self._y_data)

    def get_plot(self) -> QwtPlotCurve:
        return self._plot


class GenericDataPlot(QwtPlot):
    _GRAPH_MINW = 80
    _GRAPH_MINH = 100

    def __init__(self, data: Dict[str, List[float]], colors: str) -> None:
        '''
            Initialize the plot.
            @param data: The data to be plotted
            @param colors: The colors of the traces
        '''
        super().__init__()
        self._colors = colors
        self.setCanvasBackground(Qt.white)
        self.setMinimumSize(self._GRAPH_MINW, self._GRAPH_MINH)
        self.axisScaleDraw(QwtPlot.xBottom).enableComponent(QwtScaleDraw.Labels, False)
        self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, False)

        self._traces: Dict[str, Trace] = {}
        self.generate_plots(data)

    def generate_plots(self, data: Dict[str, List[float]]) -> None:
        '''
            Generate the plots for the data.
            @param data: The data to be plotted
        '''
        self._data_fields = list(data.keys())
        for i, data_field in enumerate(self._data_fields):
            self._traces[data_field] = Trace(data_field, QColor(self._colors[i]))

    def show_trace(self, data_field: str, show: bool=True) -> None:
        '''
            Show or hide a trace.
            @param data_field: The data field of the trace
            @param show: True if the trace should be shown, False if the trace should be hidden
        '''
        self.axisScaleDraw(QwtPlot.yLeft).enableComponent(QwtScaleDraw.Labels, True)
        if show:
            self._traces[data_field].get_plot().attach(self)
            return
        self._traces[data_field].get_plot().detach()

    def update_plot(self, data: Dict[str, Trace]) -> None:
        '''
            Update the plot with new data.
            @param data: The new data to be plotted
        '''
        for data_field in list(data.keys()):
            if self._traces[data_field].get_plot().plot():
                self._traces[data_field].update_trace_data(data[data_field])
                self.replot()

    def set_auto_scale(self) -> None:
        '''
            Set the y axis to auto scale.
        '''
        self.setAxisAutoScale(QwtPlot.yLeft)
    
    def set_trace_scale(self, data_field: str) -> None:
        '''
            Set the y axis to the minimum and maximum values of the trace.
            @param data_field: The data field of the trace
        '''
        self.setAxisScale(QwtPlot.yLeft,
                          *self._traces[data_field].get_min_max_for_axis())

    def get_data_fields(self) -> List[str]:
        '''
            Get the data fields of the plot.
            @return: The data fields of the plot
        '''
        return self._data_fields
