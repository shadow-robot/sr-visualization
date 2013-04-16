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


import os, tarfile, shutil
import roslib
roslib.load_manifest('sr_gui_self_test')
import rospy, rosgraph
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from sr_robot_msgs.srv import ManualSelfTest, ManualSelfTestResponse
from diagnostic_msgs.srv import SelfTest

from QtGui import QWidget, QTreeWidgetItem, QColor, QPixmap, QMessageBox, QInputDialog, QDialog
from QtCore import QThread, SIGNAL, QPoint
from QtCore import Qt

green = QColor(153, 231, 96)
orange = QColor(247, 206, 134)
red = QColor(236, 178, 178)

class AsyncService(QThread):
    def __init__(self, widget, node_name, index):
        """
        Calling the self test services asynchronously
        so that it doesn't "kill" the GUI while they run.
        (also runs all test services in parallel -> faster).

        @widget: parent widget
        @node_name: name of the node for which we're running the self_test
        @index: index of this thread in the list of threads (to find out which thread finished in callback)
        """
        QThread.__init__(self, widget)
        self._widget = widget
        self.node_name = node_name
        self.service_name = node_name+"/self_test"
        self.index = index

        self.srv_manual_test_ = rospy.Service(node_name+"/manual_self_tests", ManualSelfTest, self.manual_test_srv_cb_)

        self.manual_test_req_ = None
        self.manual_test_res_ = None

        self.resp = None

    def manual_test_srv_cb_(self, req):
        self.manual_test_req_ = req
        self.emit(SIGNAL("manual_test(QPoint)"), QPoint( self.index, 0))

        while self.manual_test_res_ == None:
            time.sleep(0.01)

        return self.manual_test_res_

    def run(self):
        """
        Calls the node/self_test service and emits a signal once it's finished running.
        """
        import time
        for i in range(0,100):
            time.sleep(0.1)

        self_test_srv = rospy.ServiceProxy(self.service_name, SelfTest)
        try:
            self.resp = self_test_srv()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to called " + self.service_name+" %s"%str(e))
        if self.resp == None:
            rospy.logerr("Failed to called " + self.service_name+" %s"%str(e))
            return

        self.emit(SIGNAL("test_finished(QPoint)"), QPoint( self.index, 0))

    def save(self):
        """
        Save the test results in a file at /tmp/self_tests/node/results.txt
        """
        if self.resp != None:
            path = "/tmp/self_tests/"+self.node_name
            if not os.path.exists(path):
                os.makedirs(path)
            f = open(path+"/results.txt", "w")
            f.write( str(self.resp) )
            f.close()
        else:
            rospy.logerr("Test for "+self.node_name+" can't be saved: no results found.")

    def shutdown(self):
        self.manual_test_res_ = None
        self.manual_test_req_ = None
        self.srv_manual_test_.shutdown()
        self.srv_manual_test_ = None

class SrGuiSelfTest(Plugin):
    def __init__(self, context):
        """
        Detects which nodes are advertising a self_test service, makes it possible to run them,
        and display the results.
        """
        super(SrGuiSelfTest, self).__init__(context)
        self.setObjectName('SrGuiSelfTest')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../uis/SrSelfTest.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrSelfTestUi')
        context.add_widget(self._widget)

        self.nodes = None
        self.selected_node_ = None
        self.test_threads = []

        self.index_picture = 0
        self.list_of_pics = []
        self.list_of_pics_tests = []

        self._widget.btn_test.setEnabled(False)
        self._widget.btn_save.setEnabled(False)
        self._widget.btn_prev.setEnabled(False)
        self._widget.btn_next.setEnabled(False)

        self._widget.btn_refresh_nodes.pressed.connect(self.on_btn_refresh_nodes_clicked_)
        self._widget.btn_test.pressed.connect(self.on_btn_test_clicked_)
        self._widget.btn_save.pressed.connect(self.on_btn_save_clicked_)

        self._widget.btn_next.pressed.connect(self.on_btn_next_clicked_)
        self._widget.btn_prev.pressed.connect(self.on_btn_prev_clicked_)

        self._widget.nodes_combo.currentIndexChanged.connect(self.new_node_selected_)

        self.on_btn_refresh_nodes_clicked_()


    def on_btn_save_clicked_(self):
        """
        Save the tests in a tarball.
        """
        for test in self.test_threads:
            test.save()

        #backup previous test results if they exist
        path = "/tmp/self_tests.tar.gz"
        if os.path.isfile(path):
            shutil.copy(path, path+".bk")
            os.remove(path)

        #create the tarball and save everything in it.
        tarball = tarfile.open(path, "w:gz")
        tarball.add("/tmp/self_tests")
        tarball.close()

        QMessageBox.warning(self._widget, "Information", "A tarball was saved in "+path+", please email it to hand@shadowrobot.com.")

    def on_btn_test_clicked_(self):
        """
        Run the tests in separate threads (in parallel)
        """
        #disable btn, fold previous tests and reset progress bar
        self._widget.btn_test.setEnabled(False)
        self._widget.btn_save.setEnabled(False)
        root_item = self._widget.test_tree.invisibleRootItem()
        for i in range( root_item.childCount() ):
            item = root_item.child(i)
            item.setExpanded(False)
        #also change cursor to "wait"
        self._widget.setCursor(Qt.WaitCursor)

        self._widget.progress_bar.reset()

        #delete previous results
        self.test_threads = []

        nodes_to_test = []
        if self.selected_node_ == "All":
            nodes_to_test = self.nodes[1:]
        else:
            nodes_to_test = [self.selected_node_]

        for n in nodes_to_test:
            self.test_threads.append(AsyncService(self._widget, n, len(self.test_threads)))
            self._widget.connect(self.test_threads[-1], SIGNAL("test_finished(QPoint)"), self.on_test_finished_)
            self._widget.connect(self.test_threads[-1], SIGNAL("manual_test(QPoint)"), self.on_manual_test_)

        for thread in self.test_threads:
            thread.start()

    def on_test_finished_(self, point):
        """
        Callback from test_finished signal. Displays the results and update the progress
        """
        thread = self.test_threads[point.x()]
        resp = thread.resp
        node_item = None
        if resp.passed:
            node_item = QTreeWidgetItem(["OK", thread.node_name + " ["+str(resp.id)+"]"])
            node_item.setBackgroundColor(0, QColor(green))
        else:
            node_item = QTreeWidgetItem(["FAILED", thread.node_name + " ["+str(resp.id)+"]"])
            node_item.setBackgroundColor(0, QColor(red))
        self._widget.test_tree.addTopLevelItem(node_item)

        #also display statuses
        for status in resp.status:
            display = ["", "", "", status.name, status.message]
            color = None
            if status.level == status.OK:
                display[2] = "OK"
                color = QColor(green)
            elif status.level == status.WARN:
                display[2] = "WARN"
                color = QColor(orange)
            else:
                display[2] = "ERROR"
                color = QColor(red)
            st_item = QTreeWidgetItem(node_item, display)
            st_item.setBackgroundColor(2, color)
            self._widget.test_tree.addTopLevelItem(st_item)
            st_item.setExpanded(True)
        node_item.setExpanded(True)

        #display the plots if available
        self.display_plots_(thread.node_name)

        for col in range(0, self._widget.test_tree.columnCount()):
            self._widget.test_tree.resizeColumnToContents(col)

        #display progress advancement
        nb_threads_finished = 0
        for thread in self.test_threads:
            if thread.resp is not None:
                nb_threads_finished += 1
                thread.shutdown()

        percentage = 100.0 * nb_threads_finished / len(self.test_threads)
        self._widget.progress_bar.setValue( percentage )

        if percentage == 100.0:
            #all tests were run, reenable button
            self._widget.btn_test.setEnabled(True)
            #also change cursor to standard arrow
            self._widget.setCursor(Qt.ArrowCursor)
            self._widget.btn_save.setEnabled(True)
            #empty thread list
            self.test_threads = []

    def on_manual_test_(self, point):
        thread = self.test_threads[point.x()]

        input_dialog = QInputDialog(self._widget)
        input_dialog.setOkButtonText("OK - Test successful.")
        input_dialog.setCancelButtonText("NO - Test failed (please enter a comment to say why the test fail above).")
        input_dialog.setLabelText(thread.manual_test_req_.message)
        input_dialog.setWindowTitle( thread.node_name + " - Manual Test")

        ok = (QDialog.Accepted == input_dialog.exec_())

        thread.manual_test_res_ = ManualSelfTestResponse(ok, input_dialog.textValue())

    def display_plots_(self, display_node):
        """
        Loads the plots available in /tmp/self_tests/node (place where the sr_self_test saves
        the plots for the fingers movements)
        """
        self.list_of_pics = []
        self.list_of_pics_tests = []
        for root, dirs, files in os.walk("/tmp/self_tests/"):
            for f in files:
                node_name = root.split("/")[-1]
                if node_name == display_node:
                    if ".png" in f:
                        self.list_of_pics.append(os.path.join(root,f))
                        self.list_of_pics_tests.append(node_name)

        self.index_picture = 0
        self.refresh_pic_()

    def refresh_pic_(self):
        """
        Refresh the pic being displayed
        """
        if len(self.list_of_pics) > 0:
            self._widget.label_node.setText(self.list_of_pics_tests[self.index_picture] + " ["+str(self.index_picture+1)+"/"+str(len(self.list_of_pics))+"]")
            self._widget.img.setPixmap(QPixmap(self.list_of_pics[self.index_picture]))
            self._widget.btn_prev.setEnabled(True)
            self._widget.btn_next.setEnabled(True)
        else:
            self._widget.btn_prev.setEnabled(False)
            self._widget.btn_next.setEnabled(False)
        self._widget.img.update()

    def on_btn_next_clicked_(self):
        """
        Next pic
        """
        self.index_picture = min(self.index_picture + 1, len(self.list_of_pics) - 1)
        self.refresh_pic_()

    def on_btn_prev_clicked_(self):
        """
        Prev pic
        """
        self.index_picture = max(self.index_picture - 1, 0)
        self.refresh_pic_()

    def on_btn_refresh_nodes_clicked_(self):
        """
        Refresh the list of nodes (check which node is advertising a self_test service)
        """
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
        """
        Callback for node selection dropdown
        """
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

