#!/usr/bin/env python
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import time
import rospy
import rospkg
import sys

from rospy import loginfo, logerr, logdebug

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

import QtCore
from QtCore import Qt
import QtGui
from QtGui import QIcon
import QtWidgets
from QtWidgets import *

from sr_hand.Grasp import Grasp
from sr_hand.grasps_interpoler import GraspInterpoler
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_msgs.srv import CheckIfRobotStateExistsInWarehouse as HasState
from moveit_msgs.srv import DeleteRobotStateFromWarehouse as DelState

from moveit_msgs.msg import RobotState


class JointSelecter(QtWidgets.QWidget):

    """
    Select which joints to save in a new grasp
    """

    def __init__(self, parent, all_joints):
        QtWidgets.QWidget.__init__(self, parent=parent)
        self.frame = QtWidgets.QFrame()
        self.layout = QtWidgets.QGridLayout()
        self.checkboxes = []

        col = 0
        # vectors to set the correct row in the layout for each col
        rows = [0, 0, 0, 0, 0, 0]
        joint_names = all_joints.keys()
        joint_names.sort()
        for joint in joint_names:
            if "ff" in joint.lower():
                col = 0
            elif "mf" in joint.lower():
                col = 1
            elif "rf" in joint.lower():
                col = 2
            elif "lf" in joint.lower():
                col = 3
            elif "th" in joint.lower():
                col = 4
            else:
                col = 5

            row = rows[col]
            rows[col] = row + 1
            cb = QtWidgets.QCheckBox(str(joint), self.frame)
            self.checkboxes.append(cb)
            self.layout.addWidget(cb, row, col)

        self.frame.setLayout(self.layout)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()

    def get_selected(self):
        """
        Retrieve selected joints
        """
        joints = []
        for cb in self.checkboxes:
            if cb.isChecked():
                joints.append(str(cb.text()))

        return joints

    def select_all(self):
        """
        Select all joints
        """
        for cb in self.checkboxes:
            cb.setChecked(True)

    def deselect_all(self):
        """
        Unselect all joints
        """
        for cb in self.checkboxes:
            cb.setChecked(False)


class GraspSaver(QtWidgets.QDialog):

    """
    Save a new grasp from the current joints positions.
    """

    def __init__(self, parent, all_joints, plugin_parent):
        QtWidgets.QDialog.__init__(self, parent)
        self.plugin_parent = plugin_parent
        self.all_joints = all_joints
        self.setModal(True)
        self.setWindowTitle("Save Grasp")

        self.grasp_name = ""

        self.upper_frame = QtWidgets.QFrame()
        self.upper_layout = QtWidgets.QHBoxLayout()
        label_name = QtWidgets.QLabel()
        label_name.setText("Grasp Name: ")
        name_widget = QtWidgets.QLineEdit()
        name_widget.textChanged['QString'].connect(self.name_changed)

        self.upper_layout.addWidget(label_name)
        self.upper_layout.addWidget(name_widget)
        self.upper_frame.setLayout(self.upper_layout)

        select_all_frame = QtWidgets.QFrame()
        select_all_layout = QtWidgets.QHBoxLayout()
        btn_select_all = QtWidgets.QPushButton(select_all_frame)
        btn_select_all.setText("Select All")
        select_all_layout.addWidget(btn_select_all)
        btn_select_all.clicked.connect(self.select_all)
        btn_deselect_all = QtWidgets.QPushButton(select_all_frame)
        btn_deselect_all.setText("Deselect All")
        select_all_layout.addWidget(btn_deselect_all)
        btn_deselect_all.clicked.connect(self.deselect_all)
        select_all_frame.setLayout(select_all_layout)

        self.joint_selecter = JointSelecter(self, self.all_joints)

        btn_frame = QtWidgets.QFrame()
        self.btn_ok = QtWidgets.QPushButton(btn_frame)
        self.btn_ok.setText("OK")
        self.btn_ok.setDisabled(True)
        self.btn_ok.clicked.connect(self.accept)
        btn_cancel = QtWidgets.QPushButton(btn_frame)
        btn_cancel.setText("Cancel")
        btn_cancel.clicked.connect(self.reject)

        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.btn_ok)
        btn_layout.addWidget(btn_cancel)
        btn_frame.setLayout(btn_layout)

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.upper_frame)
        self.layout.addWidget(select_all_frame)
        self.layout.addWidget(self.joint_selecter)
        self.layout.addWidget(btn_frame)

        self.setLayout(self.layout)
        self.show()

        try:
            rospy.wait_for_service("has_robot_state", 1)
        except rospy.ServiceException as e:
            QtWidgets.QMessageBox.warning(
                self, "Warning", "Could not connect to warehouse services."
                "Please make sure they're running before saving grasps.")
            rospy.logerr("Tried to save, but couldn't connecto to warehouse service: %s" % str(e))
            self.reject()

        self.has_state = rospy.ServiceProxy("has_robot_state",
                                            HasState)
        self.save_state = rospy.ServiceProxy("save_robot_state",
                                             SaveState)
        self.robot_name = self.plugin_parent.hand_commander.get_robot_name()

    def select_all(self):
        """
        Select all joints
        """
        self.joint_selecter.select_all()

    def deselect_all(self):
        """
        Unselect all joints
        """
        self.joint_selecter.deselect_all()

    def name_changed(self, name):
        self.grasp_name = name
        if self.grasp_name != "":
            self.btn_ok.setEnabled(True)
        else:
            self.btn_ok.setDisabled(True)

    def accept(self):
        """
        Save grasp for the selected joints
        """

        robot_state = RobotState()

        joints_to_save = self.joint_selecter.get_selected()
        if len(joints_to_save) == 0:
            joints_to_save = self.all_joints.keys()

        robot_state.joint_state.name = joints_to_save
        robot_state.joint_state.position = [
            self.all_joints[j] for j in joints_to_save]

        if self.has_state(self.grasp_name, self.robot_name).exists:
            ret = QtWidgets.QMessageBox.question(
                self, "State already in warehouse!",
                "There is already a pose named %s in the warehouse. Overwrite?"
                % self.grasp_name, QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)

            if QtWidgets.QMessageBox.No == ret:
                return

        self.save_state(self.grasp_name, self.robot_name, robot_state)

        try:
            self.plugin_parent.newPoseSavedSignal['QString'].emit(self.grasp_name)
        except NameError:
            pass

            QtWidgets.QDialog.accept(self)


class GraspChooser(QtWidgets.QWidget):

    """
    Choose a grasp from a list of grasps.
    """

    def __init__(self, parent, plugin_parent, title):
        QtWidgets.QWidget.__init__(self)
        self.plugin_parent = plugin_parent
        self.grasp = None
        self.title = QtWidgets.QLabel()
        self.title.setText(title)

    def draw(self):
        """
        Draw the gui and connect signals
        """
        self.frame = QtWidgets.QFrame(self)

        self.list = QtWidgets.QListWidget()
        first_item = self.refresh_list()
        self.list.itemClicked['QListWidgetItem*'].connect(self.grasp_selected)

        self.list.itemDoubleClicked['QListWidgetItem*'].connect(self.double_click)
        self.list.setViewMode(QtWidgets.QListView.ListMode)
        self.list.setResizeMode(QtWidgets.QListView.Adjust)
        self.list.setCurrentItem(first_item)
        self.grasp_selected(first_item, first_time=True)

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.list)

        #
        # SIGNALS
        #

        self.plugin_parent.newPoseSavedSignal['QString'].connect(self.refresh_list)

        self.frame.setLayout(self.layout)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()

    def double_click(self, item):
        """
        Sends new targets to the hand from a dictionary mapping the name of the joint to the value of its target
        """
        self.grasp_name = str(item.text())
        self.plugin_parent.hand_commander.move_to_named_target(self.grasp_name)

        self.plugin_parent.set_reference_grasp()

    def grasp_selected(self, item, first_time=False):
        """
        grasp has been selected with a single click
        """

        self.grasp = Grasp()
        self.grasp.grasp_name = str(item.text())
        self.grasp.joints_and_positions = self.plugin_parent.\
            hand_commander.get_named_target_joint_values(item.text())

        if not first_time:
            self.plugin_parent.set_reference_grasp()

        self.plugin_parent.to_delete = self.grasp.grasp_name

    def refresh_list(self, value=0):
        """
        refreash list of grasps
        """
        self.list.clear()
        first_item = None
        self.plugin_parent.hand_commander.refresh_named_targets()
        grasps = self.plugin_parent.hand_commander.get_named_targets()
        grasps.sort()
        for grasp_name in grasps:
            item = QtWidgets.QListWidgetItem(grasp_name)
            if first_item is None:
                first_item = item
            self.list.addItem(item)
        return first_item


class GraspSlider(QtWidgets.QWidget):

    """
    Slide from one grasp to another.
    """

    def __init__(self, parent, plugin_parent):
        QtWidgets.QWidget.__init__(self, parent)
        self.plugin_parent = plugin_parent

    def draw(self):
        """
        Draw the gui and connect signals
        """
        self.frame = QtWidgets.QFrame(self)
        label_frame = QtWidgets.QFrame(self.frame)
        from_label = QtWidgets.QLabel()
        from_label.setText("From")
        ref_label = QtWidgets.QLabel()
        ref_label.setText("Reference")
        to_label = QtWidgets.QLabel()
        to_label.setText("To")
        label_layout = QtWidgets.QHBoxLayout()
        label_layout.addWidget(from_label)
        label_layout.addWidget(ref_label)
        label_layout.addWidget(to_label)

        label_frame.setLayout(label_layout)

        self.slider = QtWidgets.QSlider()
        self.slider.setOrientation(QtCore.Qt.Horizontal)
        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setTickInterval(100)
        self.slider.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.slider.setMinimum(-100)
        self.slider.setMaximum(100)
        self.slider.valueChanged.connect(self.changeValue)

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(label_frame)
        self.layout.addWidget(self.slider)

        self.frame.setLayout(self.layout)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()

    def changeValue(self, value):
        """
        interpolate from the current grasp to new value
        """
        self.plugin_parent.interpolate_grasps(value)


class SrGuiGraspController(Plugin):

    """
    Main GraspController plugin Dock window.
    """

    newPoseSavedSignal = QtCore.pyqtSignal(str)

    def __init__(self, context):
        super(SrGuiGraspController, self).__init__(context)

        self.setObjectName('SrGuiGraspController')

        self.icon_dir = os.path.join(
            rospkg.RosPack().get_path('sr_visualization_icons'), '/icons')

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_grasp_controller'), 'uis', 'SrGuiGraspController.ui')
        self._widget = QtWidgets.QWidget()
        loadUi(ui_file, self._widget)
        if context is not None:
            context.add_widget(self._widget)
        self.current_grasp = Grasp()

        self.grasp_interpoler_1 = None
        self.grasp_interpoler_2 = None

        self.layout = self._widget.layout

        subframe = QtWidgets.QFrame()
        sublayout = QtWidgets.QVBoxLayout()

        self.hand_commander = SrHandCommander()

        self.grasp_slider = GraspSlider(self._widget, self)
        sublayout.addWidget(self.grasp_slider)

        btn_frame = QtWidgets.QFrame()
        btn_layout = QtWidgets.QHBoxLayout()

        self.btn_save = QtWidgets.QPushButton()
        self.btn_save.setText("Save")
        self.btn_save.setFixedWidth(130)
        self.btn_save.setIcon(QtGui.QIcon(self.icon_dir + '/save.png'))
        self.btn_save.clicked.connect(self.save_grasp)
        btn_layout.addWidget(self.btn_save)

        self.btn_del = QtWidgets.QPushButton()
        self.btn_del.setText("Delete")
        self.btn_del.setFixedWidth(130)
        self.btn_del.clicked.connect(self.delete_grasp)
        btn_layout.addWidget(self.btn_del)

        btn_set_ref = QtWidgets.QPushButton()
        btn_set_ref.setText("Set Reference")
        btn_set_ref.setFixedWidth(130)
        btn_set_ref.setIcon(QtGui.QIcon(self.icon_dir + '/iconHand.png'))
        btn_set_ref.clicked.connect(self.set_reference_grasp)
        btn_layout.addWidget(btn_set_ref)

        btn_frame.setLayout(btn_layout)
        sublayout.addWidget(btn_frame)
        subframe.setLayout(sublayout)

        hand_finder = HandFinder()
        if hand_finder.hand_e_available():
            selector_layout = QtWidgets.QHBoxLayout()
            selector_frame = QtWidgets.QFrame()

            selector_layout.addWidget(QtWidgets.QLabel("Select Hand"))

            self.hand_combo_box = QtWidgets.QComboBox()

            hand_parameters = hand_finder.get_hand_parameters()
            for hand_serial in self.hand_parameters.mapping.keys():
                self.hand_combo_box.addItem(hand_serial)
            # TODO(@anyone): adapt so that hand Hs are included as options in combo box

            selector_layout.addWidget(self.hand_combo_box)
            selector_frame.setLayout(selector_layout)
            sublayout.addWidget(selector_frame)

            self.hand_combo_box.activated.connect(self.hand_selected)

        self.grasp_from_chooser = GraspChooser(self._widget, self, "From: ")
        self.layout.addWidget(self.grasp_from_chooser)
        self.layout.addWidget(subframe)

        self.grasp_to_chooser = GraspChooser(self._widget, self, "To: ")
        self.layout.addWidget(self.grasp_to_chooser)

        self.grasp_slider.draw()
        self.grasp_to_chooser.draw()
        self.grasp_from_chooser.draw()

        time.sleep(0.2)

        self.set_reference_grasp()

        self.to_delete = None

    def hand_selected(self, serial):
        self.hand_commander = SrHandCommander(
            hand_parameters=self.hand_parameters,
            hand_serial=serial)
        self.refresh_grasp_lists()

    def delete_grasp(self):
        if self.to_delete is None:
            QtWidgets.QMessageBox.warning(
                self._widget, "No grasp selected!",
                "Please click a grasp name in either grasp chooser to delete.")
        else:
            ret = QtWidgets.QMessageBox.question(
                self._widget, "Delete Grasp?",
                "Are you sure you wish to delete grasp %s?"
                % self.to_delete, QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)

            if ret == QtWidgets.QMessageBox.Yes:
                del_state = rospy.ServiceProxy("delete_robot_state", DelState)
                robot_name = self.hand_commander.get_robot_name()
                try:
                    del_state(self.to_delete, robot_name)
                except rospy.ServiceException as e:
                    QtWidgets.QMessageBox.warning(
                        self._widget, "Coudn't delete",
                        "Please check warehouse services are running.")
                    rospy.logwarn("Couldn't delete state: %s" % str(e))
            self.refresh_grasp_lists()

    def refresh_grasp_lists(self):
        self.grasp_from_chooser.refresh_list()
        self.grasp_to_chooser.refresh_list()

    def shutdown_plugin(self):
        self._widget.close()
        self._widget.deleteLater()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass

    def save_grasp(self):
        all_joints = self.hand_commander.get_current_state_bounded()
        GraspSaver(self._widget, all_joints, self)

    def set_reference_grasp(self, argument=None):
        """
        Set the last commander target reference for interpolation
        """
        self.current_grasp.joints_and_positions = self.hand_commander.get_current_state()
        self.grasp_slider.slider.setValue(0)

        grasp_to = self.grasp_to_chooser.grasp
        grasp_from = self.grasp_from_chooser.grasp

        for g in [grasp_to, grasp_from]:
            for k in g.joints_and_positions.keys():
                if k not in self.hand_commander._move_group_commander._g.get_joints():
                    del(g.joints_and_positions[k])

        self.grasp_interpoler_1 = GraspInterpoler(
            self.grasp_from_chooser.grasp, self.current_grasp)
        self.grasp_interpoler_2 = GraspInterpoler(
            self.current_grasp, self.grasp_to_chooser.grasp)

    def interpolate_grasps(self, value):
        """
        interpolate grasp from the current one to the one indicated by value
        or in the opposite direction if value < 0
        hand controllers must be running and reference must be set
        """
        if self.grasp_interpoler_1 is None \
                or self.grasp_interpoler_2 is None:
            QtWidgets.QMessageBox.warning(
                self._widget, "Warning", "Could not read current grasp.\n"
                "Check that the hand controllers are running.\n"
                "Then click \"Set Reference\"")
            return
        # from -> current
        targets_to_send = dict()
        if value < 0:
            targets_to_send = self.grasp_interpoler_1.interpolate(100 + value)
        else:  # current -> to
            targets_to_send = self.grasp_interpoler_2.interpolate(value)

        self.hand_commander.move_to_joint_value_target_unsafe(targets_to_send)


if __name__ == "__main__":
    rospy.init_node("grasp_controller")
    app = QtWidgets.QApplication(sys.argv)
    ctrl = SrGuiGraspController(None)
    ctrl._widget.show()
    app.exec_()
