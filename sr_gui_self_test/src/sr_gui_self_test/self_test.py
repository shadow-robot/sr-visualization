# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os
import roslib
roslib.load_manifest('sr_gui_self_test')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

import rosgraph

from QtGui import QWidget

class SrGuiSelfTest(Plugin):

    def __init__(self, context):
        super(SrGuiSelfTest, self).__init__(context)
        self.setObjectName('SrGuiSelfTest')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../uis/SrSelfTest.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrSelfTestUi')
        context.add_widget(self._widget)

        self._widget.btn_refresh_nodes.pressed.connect(self.on_btn_refresh_nodes_clicked_)
        self._widget.btn_test.pressed.connect(self.on_btn_test_clicked_)

        self.nodes = None
        self.selected_node_ = None
        self._widget.nodes_combo.currentIndexChanged.connect(self.new_node_selected_)

    def on_btn_test_clicked_(self):
        print "TODO run self test"

    def on_btn_refresh_nodes_clicked_(self):
        self.nodes = []

        #gets all the list of services and only keep the nodes which have a self_test service
        self.nodes = [x[0].split('/')[1] for x in rosgraph.masterapi.Master("/").getSystemState()[2] if "self_test" in x[0]]

        self._widget.nodes_combo.clear()
        for node in self.nodes:
            self._widget.nodes_combo.addItem(node)

    def new_node_selected_(self, index=None):
        self.selected_node_ = self.nodes[index]

    def _unregisterPublisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._unregisterPublisher()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass

