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

from diagnostic_msgs.srv import SelfTest
from diagnostic_msgs.msg import DiagnosticStatus

import rosgraph

from QtGui import QWidget, QTreeWidgetItem

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

        self._widget.btn_test.setEnabled(False)

        self.nodes = None
        self.selected_node_ = None
        self._widget.nodes_combo.currentIndexChanged.connect(self.new_node_selected_)

        self.on_btn_refresh_nodes_clicked_()

    def on_btn_test_clicked_(self):
        nodes_to_test = []
        if self.selected_node_ == "All":
            nodes_to_test = self.nodes[1:]
        else:
            nodes_to_test = [self.selected_node_]

        #fold previous tests
        root_item = self._widget.test_tree.invisibleRootItem()
        for i in range( root_item.childCount() ):
            item = root_item.child(i)
            item.setExpanded(False)

        for n in nodes_to_test:
            #TODO: do this in a thread
            self_test_srv = rospy.ServiceProxy(n+"/self_test", SelfTest)
            resp = None
            try:
                resp = self_test_srv()
            except rospy.ServiceException, e:
                rospy.logerr("Failed to called " + n+"/self_test %s"%str(e))
            if resp == None:
                rospy.logerr("Failed to called " + n+"/self_test %s"%str(e))
                return

            node_item = None
            if resp.passed:
                node_item = QTreeWidgetItem(["OK", n+"("+str(resp.id)+")"])
            else:
                node_item = QTreeWidgetItem(["FAILED", n+"("+str(resp.id)+")"])
            self._widget.test_tree.addTopLevelItem(node_item)

            for status in resp.status:
                display = ["", "", "", status.name, status.message]
                if status.level == status.OK:
                    display[2] = "OK"
                elif status.level == status.WARN:
                    display[2] = "WARN"
                else:
                    display[2] = "ERROR"
                st_item = QTreeWidgetItem(node_item, display)
                self._widget.test_tree.addTopLevelItem(st_item)
                st_item.setExpanded(True)
            node_item.setExpanded(True)

        for col in range(0, self._widget.test_tree.columnCount()):
            self._widget.test_tree.resizeColumnToContents(col)


    def on_btn_refresh_nodes_clicked_(self):
        self.nodes = []

        #gets all the list of services and only keep the nodes which have a self_test service
        self.nodes = [x[0].split('/')[1] for x in rosgraph.masterapi.Master("/").getSystemState()[2] if "self_test" in x[0]]

        #uniquify items in the list
        self.nodes = list(set(self.nodes))

        self.nodes.insert(0, "All")

        self._widget.nodes_combo.clear()
        for node in self.nodes:
            self._widget.nodes_combo.addItem(node)

    def new_node_selected_(self, index=None):
        self.selected_node_ = self.nodes[index]

        if index != None:
            self._widget.btn_test.setEnabled(True)
        else:
            self._widget.btn_test.setEnabled(False)

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

