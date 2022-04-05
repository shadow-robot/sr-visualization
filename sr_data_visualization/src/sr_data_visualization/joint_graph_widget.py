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
    QCheckBox,
    QVBoxLayout,
    QGroupBox,
)


class JointGraph(QWidget):
    """
        Creates the joint graph widget
    """
    def __init__(self,
                 joint_name,
                 joint_data_plot,
                 row,
                 column,
                 parent=None,
                 check_box=True):
        super().__init__(parent=parent)

        self.joint_name = joint_name
        self.initial_row = row
        self.initial_column = column
        self.check_box = check_box
        self.setObjectName(self.joint_name)
        self.init_ui()
        self.joint_check_box = None
        self._create_joint_graph_widget(joint_data_plot)

    def init_ui(self):
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

    def _create_joint_graph_widget(self, joint_data_plot):
        self.check_layout = QVBoxLayout()

        if self.check_box:
            groupbox = QGroupBox()
            self.layout.addWidget(groupbox)

            self.joint_check_box = QCheckBox(self.joint_name)
            self.check_layout.addWidget(self.joint_check_box)
        else:
            groupbox = QGroupBox(self.joint_name)
            self.layout.addWidget(groupbox)

        self.joint_plot = joint_data_plot
        self.check_layout.addWidget(self.joint_plot)

        groupbox.setLayout(self.check_layout)

        self.setLayout(self.layout)

    def change_side(self, side):
        self.joint_name = self.joint_name.replace("lh", side)
        self.joint_name = self.joint_name.replace("rh", side)

        if self.joint_check_box is not None:
            self.joint_check_box.setText(self.joint_name)
