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

from python_qt_binding.QtWidgets import (
    QWidget,
    QGridLayout,
    QVBoxLayout,
)

from sr_gui_fingertip_visualization.tab_options import (
    PSTVisualizationTabOptions
)

from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64MultiArray


class GenericVisualizationTab(QWidget):
    MAX_NO_COLUMNS = 4

    def __init__(self, tab_name, parent=None):
        QWidget.__init__(self, parent=parent)

        self.tab_name = tab_name
        self.init_ui()
        self.create_full_tab()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_full_tab(self):
        self.create_tab_options()
        self.graphs_layout = QGridLayout()

    def create_tab_options(self):
        raise NotImplementedError("The function create_tab_options must be implemented")


class PSTVisualizationTab(GenericVisualizationTab):
    def __init__(self, tab_name, parent=None):
        super().__init__(tab_name, parent)

    def create_tab_options(self):
        self.tab_options = PSTVisualizationTabOptions(self.tab_name)
        self.layout.addWidget(self.tab_options)