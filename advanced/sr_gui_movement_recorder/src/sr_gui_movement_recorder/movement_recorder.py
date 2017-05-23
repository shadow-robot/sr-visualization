#!/usr/bin/env python
#
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
#

import os
import rospkg
import xml.etree.ElementTree as ET
import time
import threading
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.Qt import QTimer, QLayout, QPalette, QDoubleValidator, QIntValidator

from QtCore import Qt, pyqtSignal
from QtGui import QIcon, QColor
from QtWidgets import QWidget, QFrame, QLabel, QComboBox, QLineEdit, QPushButton, \
    QHBoxLayout, QVBoxLayout, QGridLayout, QFileDialog


from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


class Step(QWidget):

    """
    A step in a sequence of steps which compose a full movement.
    Contains add / remove steps buttons, loop control, pause / interpolation time,
    grasp type.
    """

    def __init__(self, parent, step_index, plugin_parent):
        QWidget.__init__(self, parent=parent)
        self.step_index = step_index
        self.parent = plugin_parent
        self.grasp = 0
        self.pause_time = 0
        self.interpolation_time = 2
        self.loop_to_step = -1
        self.number_of_loops = 0
        self.remaining_loops = 0

        self.widgets = []

    def draw(self):
        self.frame = QFrame(self)
        self.green = QColor(153, 231, 96)
        self.saved_palette = self.palette()
        green_palette = self.palette()
        green_palette.setBrush(QPalette.Window, self.green)
        self.frame.setPalette(green_palette)

        label_grasp = QLabel(self.frame)
        label_grasp.setText("Grasp:  ")
        self.widgets.append(label_grasp)

        self.list_grasp = QComboBox(self.frame)
        self.refresh_list()
        self.list_grasp.activated['QString'].connect(self.grasp_choosed)
        self.widgets.append(self.list_grasp)

        label_pause = QLabel(self.frame)
        label_pause.setText(" Pause Time:")
        self.widgets.append(label_pause)

        self.pause_input = QLineEdit(self.frame)
        self.pause_input.setValidator(QDoubleValidator(self))
        self.pause_input.setText("0.0")
        self.pause_input.setFixedWidth(35)
        self.pause_input.setAlignment(Qt.AlignRight)
        self.pause_input.textChanged['QString'].connect(self.pause_changed)
        self.widgets.append(self.pause_input)

        label_interp = QLabel(self.frame)
        label_interp.setText("s.   Interpolation Time:")
        self.widgets.append(label_interp)

        self.interp_input = QLineEdit(self.frame)
        self.interp_input.setValidator(QDoubleValidator(self))
        self.interp_input.setText("2.0")
        self.interp_input.setAlignment(Qt.AlignRight)
        self.interp_input.setFixedWidth(35)
        self.interp_input.textChanged['QString'].connect(self.interp_changed)
        self.widgets.append(self.interp_input)

        label_looping = QLabel(self.frame)
        label_looping.setText("s.   Loop to step:")
        self.widgets.append(label_looping)

        self.loop_input = QComboBox(self.frame)
        for i in range(0, self.step_index + 1):
            if i == 0:
                self.loop_input.addItem("None")
            else:
                self.loop_input.addItem(str(i))
        self.loop_input.activated['QString'].connect(self.choose_looping)
        self.widgets.append(self.loop_input)

        self.number_loops = QLineEdit(self.frame)
        self.number_loops.setValidator(QIntValidator(self))
        self.number_loops.setDisabled(True)
        self.number_loops.setText("0")
        self.number_loops.setAlignment(Qt.AlignRight)
        self.number_loops.setFixedWidth(35)
        # self.number_loops.setReadOnly(True)
        # self.number_loops.setStyleSheet("QWidget { background-color:
        # lightgrey }")
        self.number_loops.textChanged['QString'].connect(self.number_loops_changed)
        self.widgets.append(self.number_loops)

        label_times = QLabel(self.frame)
        label_times.setText("times. ")
        self.widgets.append(label_times)

        self.new_step_button = QPushButton(self.frame)
        self.new_step_button.setText("+")
        self.new_step_button.setFixedWidth(20)
        self.new_step_button.clicked.connect(self.add_step)
        self.widgets.append(self.new_step_button)

        self.remove_step_button = QPushButton(self.frame)
        self.remove_step_button.setText("-")
        self.remove_step_button.setFixedWidth(20)
        self.remove_step_button.clicked.connect(self.remove_step)
        self.widgets.append(self.remove_step_button)

        self.layout = QHBoxLayout()
        self.layout.setAlignment(Qt.AlignCenter)
        self.layout.setSpacing(2)
        self.layout.addWidget(label_grasp)
        self.layout.addWidget(self.list_grasp)
        self.layout.addWidget(label_pause)
        self.layout.addWidget(self.pause_input)
        self.layout.addWidget(label_interp)
        self.layout.addWidget(self.interp_input)
        self.layout.addWidget(label_looping)
        self.layout.addWidget(self.loop_input)
        self.layout.addWidget(self.number_loops)
        self.layout.addWidget(label_times)
        self.layout.addWidget(self.new_step_button)
        self.layout.addWidget(self.remove_step_button)

        self.frame.setLayout(self.layout)

        layout = QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        # self.setWidget(self.frame)
        self.show()

    def grasp_choosed(self, grasp_name):
        self.grasp = str(grasp_name)

    def pause_changed(self, pause_time):
        self.pause_time = float(pause_time)

    def interp_changed(self, interp_time):
        self.interpolation_time = float(interp_time)

    def add_step(self):
        self.parent.add_step(self.step_index)

    def number_loops_changed(self, number_loops):
        self.number_of_loops = int(number_loops)

    def choose_looping(self, looping):
        if looping == "None":
            self.number_loops.setDisabled(True)
            self.number_loops.setText("0")
            self.loop_to_step = -1
        else:
            self.number_loops.setEnabled(True)
            self.number_loops.setText("1")
            self.number_of_loops = 1
            self.loop_to_step = int(looping) - 1

    def remove_step(self, delete_first=False):
        """
        Make sure we don't delete the first item from the GUI
        """
        if not delete_first:
            if len(self.parent.steps) <= 1:
                return
        self.close()
        self.parent.steps.remove(self)
        del self

    def set_step_id(self, index):
        self.step_index = index

    def is_playing(self):
        self.frame.setAutoFillBackground(True)
        self.frame.repaint()

    def stopped_playing(self):
        self.frame.setAutoFillBackground(False)
        self.frame.repaint()

    def save_to_xml(self):
        xml_step = ET.Element("step")
        grasp = ET.SubElement(xml_step, "grasp")
        grasp.set("name", self.grasp)
        pause = ET.SubElement(xml_step, "pause_time")
        pause.text = str(self.pause_time)
        interpolation = ET.SubElement(xml_step, "interpolation_time")
        interpolation.text = str(self.interpolation_time)
        looping = ET.SubElement(xml_step, "loop_to_step")
        looping.text = str(self.loop_to_step)
        nb_loops = ET.SubElement(xml_step, "number_loops")
        nb_loops.text = str(self.number_of_loops)
        return xml_step

    def load_from_xml(self, xml_element):
        for subelement in xml_element:
            if subelement.tag == "grasp":
                grasp_name = subelement.attrib.get("name")
                self.grasp_choosed(grasp_name)
                list_grasps = self.parent.hand_commander.get_named_targets()
                list_grasps.sort()
                for index, grasp_name_ref in zip(range(0, len(list_grasps)), list_grasps):
                    if grasp_name == grasp_name_ref:
                        self.list_grasp.setCurrentIndex(index)
                        break
            if subelement.tag == "pause_time":
                self.pause_time = float(subelement.text)
                self.pause_input.setText(subelement.text)

            if subelement.tag == "interpolation_time":
                self.interpolation_time = float(subelement.text)
                self.interp_input.setText(subelement.text)

            if subelement.tag == "loop_to_step":
                self.loop_to_step = int(subelement.text)
                self.loop_input.setCurrentIndex(self.loop_to_step + 1)
                if self.loop_to_step == -1:
                    self.number_loops.setDisabled(True)
                else:
                    self.number_loops.setEnabled(True)

            if subelement.tag == "number_loops":
                self.number_of_loops = int(subelement.text)
                self.number_loops.setText(subelement.text)
        self.grasp_slider = None

    def refresh_list(self, value=0):
        self.list_grasp.clear()
        list_grasps = self.parent.hand_commander.get_named_targets()
        list_grasps.sort()

        self.grasp = list_grasps[0]

        for grasp_name in list_grasps:
            self.list_grasp.addItem(grasp_name)


class SrGuiMovementRecorder(Plugin):

    """
    A rosgui plugin for recording and replaying movements
    """

    def __init__(self, context):
        super(SrGuiMovementRecorder, self).__init__(context)
        self.setObjectName('SrGuiMovementRecorder')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_movement_recorder'), 'uis', 'SrGuiMovementRecorder.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrGuiMovementRecorderUi')
        if context is not None:
            context.add_widget(self._widget)

        self.frame = self._widget.frame
        self.timer = QTimer(self.frame)

        self.layout = QVBoxLayout()
        self.layout.setSpacing(2)

        self.layout.setAlignment(Qt.AlignCenter)
        self.layout.setSizeConstraint(QLayout.SetFixedSize)

        self.sublayout = QGridLayout()
        self.command_frame = QFrame()

        self.hand_combo_box = QComboBox()
        self.sublayout.addWidget(QLabel("Select Hand"), 0, 0)

        # setting up the hand selection
        hand_finder = HandFinder()
        if hand_finder.hand_e_available():
            self.hand_parameters = hand_finder.get_hand_parameters()

            for hand_serial in self.hand_parameters.mapping.keys():
                self.hand_combo_box.addItem(hand_serial)
        # TODO(@anyone): Make so grasp controller combo box can be used to select multiple hand H

        self.sublayout.addWidget(self.hand_combo_box, 0, 1)

        self.hand_combo_box.activated['QString'].connect(self.hand_selected)

        self.play_btn = QPushButton()
        self.play_btn.setText("Play")
        self.play_btn.setFixedWidth(80)
        self.play_btn.clicked.connect(self.button_play_clicked)
        self.sublayout.addWidget(self.play_btn, 0, 2)

        self.mutex = threading.Lock()
        self.stopped = True

        self.thread = None

        self.stop_btn = QPushButton()
        self.stop_btn.setText("Stop")
        self.stop_btn.setFixedWidth(80)
        self.stop_btn.clicked.connect(self.stop)
        self.sublayout.addWidget(self.stop_btn, 0, 3)

        self.sublayout.addWidget(QLabel(''), 0, 4)

        self.save_btn = QPushButton()
        self.save_btn.setText("Save")
        self.save_btn.setFixedWidth(80)
        self.save_btn.clicked.connect(self.save)
        self.sublayout.addWidget(self.save_btn, 0, 5)

        self.load_btn = QPushButton()
        self.load_btn.setText("Load")
        self.load_btn.setFixedWidth(80)
        self.load_btn.clicked.connect(self.load)
        self.sublayout.addWidget(self.load_btn, 0, 6)

        self.command_frame.setLayout(self.sublayout)
        self.layout.addWidget(self.command_frame)

        self.frame.setLayout(self.layout)

        path_to_icons = os.path.join(
            rospkg.RosPack().get_path('sr_visualization_icons'), 'icons')
        self.load_btn.setIcon(QIcon(os.path.join(path_to_icons, 'load.png')))
        self.save_btn.setIcon(QIcon(os.path.join(path_to_icons, 'save.png')))
        self.stop_btn.setIcon(QIcon(os.path.join(path_to_icons, 'stop.png')))
        self.play_btn.setIcon(QIcon(os.path.join(path_to_icons, 'play.png')))

        self.hand_commander = None
        self.steps = []

        self.steps_frame = QFrame()
        self.steps_layout = QVBoxLayout()
        self.steps_frame.setLayout(self.steps_layout)

        self.layout.addWidget(self.steps_frame)

        # selecting the first available hand
        self.__selected_hand = None
        # TODO(@dg-shadow) Fix hand finder etc to work with hand H
        if hand_finder.hand_e_available():
            self.hand_selected(self.hand_parameters.mapping.keys()[0])
        else:
            self.hand_commander = SrHandCommander()
            self.remove_all_steps()
            self.add_step()
            self.__selected_hand = self.hand_commander.get_group_name()

    def hand_selected(self, serial):
        self.hand_commander = SrHandCommander(hand_parameters=self.hand_parameters,
                                              hand_serial=serial)

        if self.__selected_hand == serial:
            return

        self.remove_all_steps()
        self.add_step()
        self.__selected_hand = serial

    def save(self):
        filename = QFileDialog.getSaveFileName(self.frame, 'Save Script', '')
        filename = filename[0]

        if filename == "":
            return

        root = ET.Element("movement")
        for step in self.steps:
            root.append(step.save_to_xml())

        self.indent(root)
        tree = ET.ElementTree(root)

        tree.write(filename)

    def indent(self, elem, level=0):
        """
        print a prettier / indented xml tree
        """
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.indent(elem, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def load(self):
        """
        remove all the present steps
        and load new from xml file
        """
        filename = QFileDialog.getOpenFileName(self.frame, 'Open Script', '')
        filename = filename[0]

        if filename != "":
            self.remove_all_steps()

            tree = ET.parse(filename)
            root = tree.getroot()
            xml_steps = tree.findall("step")

            for step in xml_steps:
                self.add_step()
                self.steps[-1].load_from_xml(step)

    def remove_all_steps(self):
        while len(self.steps) != 0:
            self.steps[0].remove_step(delete_first=True)

    def add_step(self, step_index=None):
        if step_index is None:
            step_index = len(self.steps)
        else:
            step_index += 1

        step_tmp = Step(self.frame, step_index, self)
        self.steps.insert(step_index, step_tmp)

        for index, step in enumerate(self.steps):
            step.set_step_id(index)

        self.steps_layout.insertWidget(step_index, step_tmp)
        step_tmp.draw()

    def button_play_clicked(self):
        if len(self.steps) < 1:
            return

        self.play_btn.setDisabled(True)
        self.load_btn.setDisabled(True)

        for step in self.steps:
            step.remove_step_button.setDisabled(True)
            step.new_step_button.setDisabled(True)
            step.remaining_loops = step.number_of_loops

        self.fill_trajectory()

        self.thread = threading.Thread(None, self.play)
        self.thread.start()

    def fill_trajectory(self, step_index=None):
        if step_index is None:
            self.trajectory = []
            step_index = 0
        elif step_index >= len(self.steps):
            return

        step = self.steps[step_index]

        self.trajectory.append(
            {
                'name': step.grasp,
                'interpolate_time': step.interpolation_time,
                'pause_time': step.pause_time
            }
        )

        if step.loop_to_step != -1 and step.remaining_loops > 0:
            step.remaining_loops -= 1
            next_step = step.loop_to_step
        else:
            next_step = step_index + 1
        self.fill_trajectory(next_step)

    def play(self):
        self.stopped = False
        first_time = True
        index = 0

        run = True
        in_progress = False
        while run:
            self.mutex.acquire()
            if self.stopped:
                self.mutex.release()
                return
            self.mutex.release()

            if not in_progress:
                self.hand_commander.run_named_trajectory_unsafe(self.trajectory)
                in_progress = True
            else:
                rospy.sleep(0.1)
                run = self.hand_commander.action_is_running()

        self.stop()

    def stop(self):
        self.hand_commander.send_stop_trajectory_unsafe()
        for step in self.steps:
            step.new_step_button.setEnabled(True)
            step.remove_step_button.setEnabled(True)
        self.mutex.acquire()
        self.stopped = True
        self.mutex.release()
        self.play_btn.setEnabled(True)
        self.load_btn.setEnabled(True)

    def _unregisterPublisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self.remove_all_steps()
        self._unregisterPublisher()

if __name__ == "__main__":
    from QtWidgets import QApplication
    import sys
    rospy.init_node("movement_recorder")
    app = QApplication(sys.argv)
    ctrl = SrGuiMovementRecorder(None)
    ctrl._widget.show()
    app.exec_()