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

from PyQt5.QtWidgets import (
    QWidget,
    QGridLayout,
    QCheckBox,
    QVBoxLayout,
    QGroupBox,
)


class JointGraph(QWidget):
    """
        Creates the joint graph widget
    """
    def __init__(self, joint_name, joint_data_plot, row, column, parent=None):
        QWidget.__init__(self, parent=parent)

        self.joint_name = joint_name
        self.initial_row = row
        self.initial_column = column
        self.setObjectName(self.joint_name)
        self.init_ui()
        self._create_joint_graph_widget(joint_data_plot)

    def init_ui(self):
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

    def _create_joint_graph_widget(self, joint_data_plot):
        groupbox = QGroupBox()
        self.layout.addWidget(groupbox)

        self.check_layout = QVBoxLayout()
        groupbox.setLayout(self.check_layout)

        self.joint_check_box = QCheckBox(self.joint_name)
        self.joint_plot = joint_data_plot
        self.check_layout.addWidget(self.joint_check_box)
        self.check_layout.addWidget(self.joint_plot)

        self.setLayout(self.layout)
