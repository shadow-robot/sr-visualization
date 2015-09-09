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

#import roslib; roslib.load_manifest('sr_control_gui')
import os, rospy, rospkg

import traceback
import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import QtCore
from QtCore import Qt, QEvent, QObject
import QtGui
from QtGui import *

#from cyberglove_generic_plugin import CybergloveGenericPlugin

from cyberglove_calibrer import *
from cyberglove_mapper import *
#from cyberglove_library import Cyberglove

rootPath = os.path.join(rospkg.RosPack().get_path('sr_gui_cyberglove_calibrator'))
noimage_path = rootPath + '/images/image-missing.png'

class StepDescription():
    """
    Stores the description / images path for a given step.
    """
    def __init__(self):
        self.text = ""
        self.image_path = [noimage_path,noimage_path]
        self.current_substep = 0

class StepDescriber(QtGui.QWidget):
    """
    Displays the description / images for the current step.
    """
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent=parent)
	
        self.description = StepDescription
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()

        self.text_description = QtGui.QTextEdit()
        self.text_description.setMaximumHeight(60)
        self.layout.addWidget(self.text_description)

        self.image_description = QtGui.QLabel()
        self.image_description.setMinimumSize(170, 170)
        self.layout.addWidget(self.image_description)

        self.frame.setLayout(self.layout)
        layout = QtGui.QHBoxLayout()
        layout.addWidget(self.frame)
        self.setLayout(layout)
        self.show()

    def set_description(self, description):
        self.text_description.setText(description.text)
        index = description.current_substep
        self.image_description.setPixmap(QtGui.QPixmap(description.image_path[index]))
        self.image_description.repaint()
        self.repaint()



class StepSelector(QtGui.QWidget):
    """
    The user can select the step to calibrate through this widget.
    """
    def __init__(self, parent, calibrer):
        QtGui.QWidget.__init__(self, parent=parent)

        self.current_step_name = None
        self.current_row = 0
        self.steps = {}
        self.steps_description = {}

        self.frame = QtGui.QFrame()
        self.calibrer = calibrer
        self.layout = QtGui.QVBoxLayout()
        self.layout.setSpacing(5)

        self.title = QtGui.QLabel()
        self.title.setText("Calibration Steps - 2 substeps by steps")

        self.step_describer = StepDescriber(self)

        self.list = QtGui.QListWidget()
        first_item = self.refresh_list()
        self.connect(self.list, QtCore.SIGNAL('itemClicked(QListWidgetItem*)'), self.step_choosed)
        self.list.setViewMode(QtGui.QListView.ListMode)
        self.list.setResizeMode(QtGui.QListView.Adjust)

        self.list.setCurrentRow(0)
        first_item = self.list.item(0)
        self.list.setItemSelected(first_item, True)
        self.step_choosed(first_item, second_substep = True)

        self.layout.addWidget(self.title)
        self.layout.addWidget(self.list)

        self.frame.setLayout(self.layout)
        layout = QtGui.QHBoxLayout()
        layout.addWidget(self.frame)
        layout.addWidget(self.step_describer)
        self.setLayout(layout)
        self.show()

    def step_choosed(self, item, first_time=False, second_substep=False):
        step_name = str(item.text())
        self.current_step_name = step_name

        if not second_substep:
            self.steps_description[step_name].current_substep = 0

        self.current_row = self.list.currentRow()
        index = self.steps_description[self.current_step_name].current_substep
        name = self.current_step_name

        description = self.steps[name].step_description[index]
        self.steps_description[self.current_step_name].text = description
        self.step_describer.set_description(self.steps_description[self.current_step_name])


    def refresh_list(self, value=0):
        self.list.clear()
        first_item = None
        steps = self.calibrer.calibration_steps
        index = 1
        base_image_path = rootPath + '/images/step'
        for step in steps:
            item = QtGui.QListWidgetItem(step.step_name)
            if first_item == None:
                first_item = item
            self.list.addItem(item)
            self.steps[step.step_name] = step

            description = StepDescription()
            description.image_path = [base_image_path+str(index)+"-a.jpeg", base_image_path+str(index)+"-b.jpeg"]
            self.steps_description[step.step_name] = description
            index = index + 1
        return first_item


    def calibrate_current_step(self):
        if self.steps_description[self.current_step_name].current_substep == 0:
            self.steps_description[self.current_step_name].current_substep = 1
            self.calibrer.do_step_min(self.current_row)

            description = self.steps[self.current_step_name].step_description[1]
            self.steps_description[self.current_step_name].text = description
            self.step_describer.set_description(self.steps_description[self.current_step_name])

        elif self.steps_description[self.current_step_name].current_substep == 1:
            self.calibrer.do_step_max(self.current_row)
            if self.current_row < len(self.steps) - 1:
                self.list.setCurrentRow(self.current_row + 1)
                next_item = self.list.item(self.current_row + 1)
                self.step_choosed(next_item, second_substep = True)


class GloveCalibratingWidget(QtGui.QWidget):
    """
    Displays which joints have been calibrated.
    """
    def __init__(self, parent, joint_names):
        QtGui.QWidget.__init__(self, parent=parent)
        self.frame = QtGui.QFrame()

        self.layout = QtGui.QGridLayout()
        self.layout.setHorizontalSpacing(5)
        self.layout.setVerticalSpacing(5)

        green = QtGui.QColor(126, 255, 0)
        red = QtGui.QColor(255, 36, 0)
        orange = QtGui.QColor(255, 138, 0)
        self.saved_palette = self.palette()
        self.green_palette = self.palette()
        self.green_palette.setBrush(QtGui.QPalette.Window, green)
        self.red_palette = self.palette()
        self.red_palette.setBrush(QtGui.QPalette.Window, red)
        self.orange_palette = self.palette()
        self.orange_palette.setBrush(QtGui.QPalette.Window, orange)

        col = 0
        #vectors to set the correct row in the layout for each col
        rows = [0, 0, 0, 0, 0, 0]

        self.joints_frames = {}

        for joint in joint_names:
            if "index" in joint.lower():
                col = 0
            elif "middle" in joint.lower():
                col = 1
            elif "ring" in joint.lower():
                if "pinkie" in joint.lower():
                    col = 3
                else:
                    col = 2
            elif "pinkie" in joint.lower():
                col = 3
            elif "thumb" in joint.lower():
                col = 4
            else:
                col = 5

            row = rows[col]
            rows[col] = row + 1

            subframe = QtGui.QFrame()
            layout = QtGui.QHBoxLayout()
            name = QtGui.QLabel()
            name.setText(joint)
            layout.addWidget(name)
            subframe.setLayout(layout)
            subframe.setPalette(self.red_palette)
            subframe.setAutoFillBackground(True)
            subframe.repaint()
            self.joints_frames[joint] = subframe
            self.layout.addWidget(subframe, row, col)

        self.set_not_calibrated(joint_names)

        self.frame.setLayout(self.layout)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()

    def set_not_calibrated(self, joints):
        for joint in joints:
            self.joints_frames[joint].setPalette(self.red_palette)
            self.frame.repaint()

    def set_half_calibrated(self, joints):
        for joint in joints:
            self.joints_frames[joint].setPalette(self.orange_palette)
            self.frame.repaint()

    def set_calibrated(self, joints):
        for joint in joints:
            self.joints_frames[joint].setPalette(self.green_palette)
            self.frame.repaint()

#class CybergloveCalibratorPlugin(CybergloveGenericPlugin):
class SrGuiCybergloveCalibrator(Plugin):
    """
    The plugin used to calibrate the glove.
    """
    name = "Cyberglove Calibrator"
    def __init__(self, context):
		super(SrGuiCybergloveCalibrator, self).__init__(context)
		self.setObjectName('SrGuiCybergloveCalibrator')
		self.icon_dir = os.path.join(rospkg.RosPack().get_path('sr_visualization_icons'), '/icons')
		
		ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_cyberglove_calibrator'), 'uis', 'SrGuiCybergloveCalibrator.ui')
		self._widget = QWidget()
		loadUi(ui_file, self._widget)
		context.add_widget(self._widget)
		
		#self.frame = QtGui.QFrame()
		#self.layout = QtGui.QVBoxLayout()
		#self.frame.setLayout(self.layout)
		#self.window.setWidget(self.frame)

		self.calibrer = CybergloveCalibrer(description_function = None)
		self.joint_names = self.calibrer.cyberglove.joints.keys()
		self.joint_names.sort()
		
		self.layout = self._widget.layout
		subframe = QtGui.QFrame()
		sublayout = QtGui.QHBoxLayout()
		
		self.glove_calibrating_widget = GloveCalibratingWidget(self._widget, self.joint_names)
		self.layout.addWidget(self.glove_calibrating_widget)
		
		self.step_selector = StepSelector(self._widget, self.calibrer)
		sublayout.addWidget(self.step_selector)
		
		btn_frame = QtGui.QFrame()
		btn_layout = QtGui.QVBoxLayout()
		btn_layout.setSpacing(25)
		btn_calibrate = QtGui.QPushButton()
		btn_calibrate.setText("Calibrate")
		btn_calibrate.setToolTip("Calibrate the current selected step")
		btn_calibrate.setIcon(QtGui.QIcon(rootPath + '/images/icons/calibrate.png'))
		btn_layout.addWidget(btn_calibrate)
		btn_frame.connect(btn_calibrate, QtCore.SIGNAL('clicked()'), self.calibrate_current_step)
		self.btn_save = QtGui.QPushButton()
		self.btn_save.setText("Save")
		self.btn_save.setToolTip("Save the current calibration")
		self.btn_save.setIcon(QtGui.QIcon(rootPath + '/images/icons/save.png'))
		self.btn_save.setDisabled(True)
		btn_layout.addWidget(self.btn_save)
		btn_frame.connect(self.btn_save, QtCore.SIGNAL('clicked()'), self.save_calib)
		btn_load = QtGui.QPushButton()
		btn_load.setText("Load")
		btn_load.setToolTip("Load a Glove calibration")
		btn_load.setIcon(QtGui.QIcon(rootPath + '/images/icons/load.png'))
		btn_layout.addWidget(btn_load)
		btn_frame.connect(btn_load, QtCore.SIGNAL('clicked()'), self.load_calib)
		btn_frame.setLayout(btn_layout)
		sublayout.addWidget(btn_frame)
		subframe.setLayout(sublayout)
		self.layout.addWidget(subframe)
	
	#	QtCore.QTimer.singleShot(0, self.window.adjustSize)

    def calibrate_current_step(self):
        self.step_selector.calibrate_current_step()

        for name in self.joint_names:
            if self.calibrer.is_step_done(name) == 0.5:
                self.glove_calibrating_widget.set_half_calibrated([name])
            elif self.calibrer.is_step_done(name) == 1.0:
                self.glove_calibrating_widget.set_calibrated([name])

        if self.calibrer.all_steps_done():
            self.btn_save.setEnabled(True)

    def save_calib(self):
        filename = QtGui.QFileDialog.getSaveFileName(self._widget, 'Save Calibration', '')
        if filename == "":
            return
       
        self.calibrer.write_calibration_file(filename)

        # QMessageBox returns 0 for yes
        if QtGui.QMessageBox.information(self._widget,
                                         "Load new Calibration",
                                         "Do you want to load the new calibration file?",
                                         "yes",
                                         "no") == 0:
            self.load_calib(filename)



    def load_calib(self, filename = ""):
        if filename == "":
            filename = QtGui.QFileDialog.getOpenFileName(self._widget, 'Open Calibration', '')
            if filename == "":
                return

        self.calibrer.load_calib(str(filename))

        self.emit(QtCore.SIGNAL("messageToStatusbar(QString)"),
                         "New Cyberglove Calibration Loaded.")
