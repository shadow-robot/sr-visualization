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

import os, subprocess, math, time
import rospy, rosparam, rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import QEvent, QObject, Qt, QTimer, Slot, SIGNAL, QThread
from QtGui import QWidget, QTreeWidgetItem, QCheckBox, QSpinBox, QDoubleSpinBox, QFileDialog, QMessageBox, QPushButton, QFrame, QHBoxLayout
from functools import partial
from tempfile import NamedTemporaryFile

from sensor_msgs.msg import JointState

from sr_gui_controller_tuner.sr_controller_tuner import SrControllerTunerApp

class PlotThread(QThread):
    def __init__(self, parent = None, joint_name = "FFJ0", controller_type = "Motor Force"):
        QThread.__init__(self, parent)
        self.joint_name_ = joint_name
        self.is_joint_0_ = False
        self.joint_index_in_joint_state_ = None
        try:
            self.joint_index_in_joint_state_ = ["FFJ0", "MFJ0", "RFJ0", "LFJ0"].index(self.joint_name_)
            self.is_joint_0_ = True
        except ValueError:
            self.is_joint_0_ = False

        self.controller_type_ = controller_type

        #prepares the title for the plot (can't contain spaces)
        self.plot_title_ = self.joint_name_ + " " + self.controller_type_
        self.plot_title_ = self.plot_title_.replace(" ", "_").replace("/","_")

        #stores the subprocesses to be able to terminate them on close
        self.subprocess_ = []

    def run(self):
        """
        Creates an appropriate plot according to controller type
        Also creates a subscription if controller is of Motor Force type
        """
        #rxplot_str = "rxplot -b 30 -p 30 --title=" + self.plot_title_ + " "
        rxplot_str = "rosrun rqt_plot rqt_plot " 

        if self.controller_type_ == "Motor Force":
            # the joint 0s are published on a different topic: /joint_0s/joint_states
            if not self.is_joint_0_:
                #subscribe to joint_states to get the index of the joint in the message
                self.subscriber_ = rospy.Subscriber("joint_states", JointState, self.js_callback_)

                #wait until we got the joint index in the joint
                # states message
                while self.joint_index_in_joint_state_ == None:
                    time.sleep(0.01)

                rxplot_str += "joint_states/effort["+ str(self.joint_index_in_joint_state_) +"]"
                
            else:
                rxplot_str += "joint_0s/joint_states/effort["+ str(self.joint_index_in_joint_state_) +"]"
                pass

        elif self.controller_type_ == "Position":
            rxplot_str += "sh_"+self.joint_name_.lower()+"_position_controller/state/set_point,sh_"+self.joint_name_.lower()+"_position_controller/state/process_value sh_" + self.joint_name_.lower()+"_position_controller/state/command"
        elif self.controller_type_ == "Muscle Position":
            rxplot_str += "sh_"+self.joint_name_.lower()+"_muscle_position_controller/state/set_point,sh_"+self.joint_name_.lower()+"_muscle_position_controller/state/process_value sh_" + self.joint_name_.lower()+"_muscle_position_controller/state/pseudo_command sh_" + self.joint_name_.lower()+"_muscle_position_controller/state/valve_muscle_0,sh_" + self.joint_name_.lower()+"_muscle_position_controller/state/valve_muscle_1"
        elif self.controller_type_ == "Velocity":
            rxplot_str += "sh_"+self.joint_name_.lower()+"_velocity_controller/state/set_point,sh_"+self.joint_name_.lower()+"_velocity_controller/state/process_value"
        elif self.controller_type_ == "Mixed Position/Velocity":
            rxplot_str += "sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller/state/set_point,sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller/state/process_value sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller/state/process_value_dot,sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller/state/commanded_velocity sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller/state/command,sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller/state/measured_effort,sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller/state/friction_compensation"
        elif self.controller_type_ == "Effort":
            rxplot_str += "sh_"+self.joint_name_.lower()+"_effort_controller/state/set_point,sh_"+self.joint_name_.lower()+"_effort_controller/state/process_value"

        self.subprocess_.append( subprocess.Popen(rxplot_str.split()) )

    def js_callback_(self, msg):
        #get the joint index once, then unregister
        self.joint_index_in_joint_state_ = msg.name.index( self.joint_name_.upper() )
        self.subscriber_.unregister()
        self.subscriber_ = None

    def __del__(self):
        for subprocess in self.subprocess_:
            #killing the rxplot to close the window
            subprocess.kill()

        if self.subscriber_ != None:
            self.subscriber_.unregister()
            self.subscriber_ = None

        self.wait()

class MoveThread(QThread):
    def __init__(self, parent = None, joint_name = "FFJ0", controller_type = "Motor Force"):
        QThread.__init__(self, parent)
        self.joint_name_ = joint_name
        self.controller_type_ = controller_type
        self.btn = parent
        self.subprocess_ = []

    def run(self):
        self.launch_()

    def __del__(self):
        for subprocess in self.subprocess_:
            subprocess.terminate()
        self.wait()

    def create_launch_file_(self):
        """
        Create a launch file dynamically
        """
        #Not using automatic move for velocity and effort controllers
        controller_name_ = ""
        if self.controller_type_ == "Position":
            controller_name_ = "sh_"+self.joint_name_.lower()+"_position_controller"
        if self.controller_type_ == "Muscle Position":
            controller_name_ = "sh_"+self.joint_name_.lower()+"_muscle_position_controller"
        elif self.controller_type_ == "Mixed Position/Velocity":
            controller_name_ = "sh_"+self.joint_name_.lower()+"_mixed_position_velocity_controller"

        min_max = self.get_min_max_()
        ns = rospy.get_namespace()

        string = "<launch> <node ns=\"" + ns + "\" pkg=\"sr_movements\" name=\"sr_movements\" type=\"sr_movements\">"
        string += "<remap from=\"~targets\" to=\""+ controller_name_ +"/command\"/>"
        string += "<remap from=\"~inputs\" to=\""+ controller_name_ +"/state\"/>"
        string += "<param name=\"image_path\" value=\"$(find sr_movements)/movements/test.png\"/>"
        string += "<param name=\"min\" value=\""+ str(min_max[0]) +"\"/>"
        string += "<param name=\"max\" value=\""+ str(min_max[1]) + "\"/>"
        string += "<param name=\"publish_rate\" value=\"100\"/>"
        string += "<param name=\"repetition\" value=\"1000\"/>"
        string += "<param name=\"nb_step\" value=\"10000\"/>"
        string += "<param name=\"msg_type\" value=\"sr\"/>"
        string += "</node> </launch>"

        tmp_launch_file = NamedTemporaryFile(delete=False)

        tmp_launch_file.writelines(string)
        tmp_launch_file.close()

        return tmp_launch_file.name

    def get_min_max_(self):
        """
        Retrieve joint limits in radians for joint in self.joint_name 
        """
        if self.joint_name_ in ["FFJ0", "MFJ0", "RFJ0", "LFJ0"]:
            return [0.0, math.radians(180.0)]
        elif self.joint_name_ in ["FFJ3", "MFJ3", "RFJ3", "LFJ3", "THJ1"]:
            return [0.0, math.radians(90.0)]
        elif self.joint_name_ in ["LFJ5"]:
            return [0.0, math.radians(45.0)]
        elif self.joint_name_ in ["FFJ4", "MFJ4", "RFJ4", "LFJ4"]:
            return [math.radians(-20.0), math.radians(20.0)]
        elif self.joint_name_ in ["THJ2"]:
            return [math.radians(-40.0), math.radians(40.0)]
        elif self.joint_name_ in ["THJ3"]:
            return [math.radians(-15.0), math.radians(15.0)]
        elif self.joint_name_ in ["THJ4"]:
            return [math.radians(0.0), math.radians(70.0)]
        elif self.joint_name_ in ["THJ5"]:
            return [math.radians(-60.0), math.radians(60.0)]
        elif self.joint_name_ in ["WRJ1"]:
            return [math.radians(-30.0), math.radians(45.0)]
        elif self.joint_name_ in ["WRJ2"]:
            return [math.radians(-30.0), math.radians(10.0)]

    def launch_(self):
        """
        launch a dynamically created launch file
        """
        filename = self.create_launch_file_()

        launch_string = "roslaunch "+filename

        self.subprocess_.append( subprocess.Popen(launch_string.split()) )

class SrGuiControllerTuner(Plugin):
    """
    a rosgui plugin for tuning the sr_mechanism_controllers
    """
    def __init__(self, context):
        super(SrGuiControllerTuner, self).__init__(context)
        self.setObjectName('SrGuiControllerTuner')

        self.controller_type = None

        self._publisher = None
        self._widget = QWidget()

        self.file_to_save = None

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_controller_tuner'), 'uis', 'SrGuiControllerTuner.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrControllerTunerUi')
        context.add_widget(self._widget)

        #stores the movements threads to be able to stop them
        self.move_threads = []

        #stores the controllers in the same order as the dropdown
        self.controllers_in_dropdown = []

        #stores a dictionary of the widgets containing the data for
        # the different controllers.
        self.ctrl_widgets = {}

        #a library which helps us doing the real work.
        self.sr_controller_tuner_app_ = SrControllerTunerApp( os.path.join(rospkg.RosPack().get_path('sr_gui_controller_tuner'), 'data', 'controller_settings.xml') )

        #refresh the controllers once
        self.on_btn_refresh_ctrl_clicked_()
        #attach the button pressed to its action
        self._widget.btn_refresh_ctrl.pressed.connect(self.on_btn_refresh_ctrl_clicked_)
        self._widget.dropdown_ctrl.currentIndexChanged.connect(self.on_changed_controller_type_)

        self._widget.btn_save_selected.pressed.connect(self.on_btn_save_selected_clicked_)
        self._widget.btn_save_all.pressed.connect(self.on_btn_save_all_clicked_)
        self._widget.btn_select_file_path.pressed.connect(self.on_btn_select_file_path_clicked_)

        self._widget.btn_set_selected.pressed.connect(self.on_btn_set_selected_clicked_)
        self._widget.btn_set_all.pressed.connect(self.on_btn_set_all_clicked_)

        self._widget.btn_load.pressed.connect(self.on_btn_load_clicked_)
        self._widget.btn_stop_mvts.pressed.connect(self.on_btn_stop_mvts_clicked_)

    def on_btn_plot_pressed_(self, joint_name, btn):
        plot_thread = PlotThread(btn, joint_name, self.controller_type)
        plot_thread.start()

    def on_btn_move_pressed_(self, joint_name, btn):
        move_thread = MoveThread(btn, joint_name, self.controller_type)
        move_thread.start()
        self.move_threads.append(move_thread)

    def on_changed_controller_type_(self, index = None):
        """
        When controller type is changed clear the chosen file path and refresh the tree with the controller settings
        """
        if index == None:
            return
        self.reset_file_path()
        self.refresh_controller_tree_( self.controllers_in_dropdown[index] )
        
    def reset_file_path(self):
        """
        Clear the chosen file path and disable the save button until user selects another path
        """
        self._widget.txt_file_path.setText("")

        self._widget.btn_load.setEnabled(False)

        #disable the save buttons (until a new file has
        # been chosen by the user)
        self._widget.btn_save_all.setEnabled(False)
        self._widget.btn_save_selected.setEnabled(False)

    def on_btn_select_file_path_clicked_(self):
        """
        Perform controller tuning and save settings to user specified file
        sr_config stack must be installed
        """
        path_to_config = "~"
        try:
            path_to_config = os.path.join(rospkg.RosPack().get_path('sr_ethercat_hand_config'))
        except:
            rospy.logwarn("couldn't find the sr_ethercat_hand_config package, do you have the sr_config stack installed?")

        #Reading the param that contains the config_dir suffix that we should use for this hand (e.g. '' normally for a right hand  or 'lh' if this is for a left hand)
        config_subdir = rospy.get_param('config_dir', '')
        subpath = "/controls/host/" + config_subdir
        if self.sr_controller_tuner_app_.edit_only_mode:
            filter_files = "*.yaml"
        else:
            if self.controller_type == "Motor Force":
                filter_files = "*controllers.yaml"
            else:
                if self.sr_controller_tuner_app_.control_mode == "PWM":
                    filter_files = "*s_PWM.yaml"
                else:                    
                    filter_files = "*s.yaml"
                
        if self.controller_type == "Motor Force":
            filter_files = "Config (*motor"+filter_files+")"
            subpath = "/controls/motors/" + config_subdir
        elif self.controller_type == "Position":
            filter_files = "Config (*position_controller"+filter_files+")"
        elif self.controller_type == "Muscle Position":
            filter_files = "Config (*muscle_joint_position_controller"+filter_files+")"
        elif self.controller_type == "Velocity":
            filter_files = "Config (*velocity_controller"+filter_files+")"
        elif self.controller_type == "Mixed Position/Velocity":
            filter_files = "Config (*position_velocity"+filter_files+")"
        elif self.controller_type == "Effort":
            filter_files = "Config (*effort_controller"+filter_files+")"

        path_to_config += subpath

        filename, _ = QFileDialog.getOpenFileName(self._widget.tree_ctrl_settings, self._widget.tr('Save Controller Settings'),
                                                  self._widget.tr(path_to_config),
                                                  self._widget.tr(filter_files))

        if filename == "":
            return

        self.file_to_save = filename

        self._widget.txt_file_path.setText(filename)

        self._widget.btn_load.setEnabled(True)

        #enable the save buttons once a file has
        # been chosen
        self._widget.btn_save_all.setEnabled(True)
        self._widget.btn_save_selected.setEnabled(True)

    def on_btn_load_clicked_(self):
        """
        reload the parameters in rosparam, then refresh the tree widget
        """
        paramlist = rosparam.load_file( self.file_to_save )
        for params,ns in paramlist:
            rosparam.upload_params(ns, params)

        self.refresh_controller_tree_( self.controllers_in_dropdown[self._widget.dropdown_ctrl.currentIndex()] )

    def on_btn_save_selected_clicked_(self):
        """
        Save only the selected controllers
        """
        selected_items = self._widget.tree_ctrl_settings.selectedItems()

        if len( selected_items ) == 0:
            QMessageBox.warning(self._widget.tree_ctrl_settings, "Warning", "No motors selected.")

        for it in selected_items:
            if str(it.text(1)) != "":
                self.save_controller( str(it.text(1)) )

    def on_btn_save_all_clicked_(self):
        """
        Save all controllers
        """
        for motor in self.ctrl_widgets.keys():
            self.save_controller( motor )

    def on_btn_set_selected_clicked_(self):
        """
        Sets the current values for selected controllers using the ros service.
        """
        selected_items = self._widget.tree_ctrl_settings.selectedItems()

        if len( selected_items ) == 0:
            QMessageBox.warning(self._widget.tree_ctrl_settings, "Warning", "No motors selected.")

        for it in selected_items:
            if str(it.text(1)) != "":
                self.set_controller( str(it.text(1)) )

    def on_btn_set_all_clicked_(self):
        """
        Sets the current values for all controllers using the ros service.
        """
        for motor in self.ctrl_widgets.keys():
            self.set_controller( motor )

    def on_btn_refresh_ctrl_clicked_(self):
        """
        Calls refresh_controller_tree_ after preparing widgets
        """
        ctrls = self.sr_controller_tuner_app_.get_ctrls()
        self.sr_controller_tuner_app_.refresh_control_mode()
        self.controllers_in_dropdown = []
        self._widget.dropdown_ctrl.clear()
        for ctrl in ctrls:
            self._widget.dropdown_ctrl.addItem(ctrl)
            self.controllers_in_dropdown.append(ctrl)

        self.refresh_controller_tree_()

    def read_settings(self, joint_name):
        """
        retrive settings for joint with given name
        """
        dict_of_widgets = self.ctrl_widgets[joint_name]

        settings = {}
        for item in dict_of_widgets.items():
            if item[0] == "sign":
                if item[1].checkState() == Qt.Checked:
                    settings["sign"] = 1
                else:
                    settings["sign"] = 0
            else:
                try:
                    settings[item[0]] = item[1].value()
                except AttributeError:
                    pass

        return settings

    def set_controller(self, joint_name):
        """
        Sets the current values for the given controller using the ros service.
        """
        settings = self.read_settings( joint_name )

        #uses the library to call the service properly
        success = self.sr_controller_tuner_app_.set_controller(joint_name, self.controller_type, settings)
        if success == False:
            if self.controller_type == "Motor Force":
                QMessageBox.warning(self._widget.tree_ctrl_settings, "Warning", "Failed to set the PID values for joint "+ joint_name +". This won't work for Gazebo controllers as there are no force controllers yet.")
            else:
                QMessageBox.warning(self._widget.tree_ctrl_settings, "Warning", "Failed to set the PID values for joint "+ joint_name +".")


    def save_controller(self, joint_name):
        """
        Saves the current values for the given controller using the ros service.
        """
        settings = self.read_settings( joint_name )

        #uses the library to call the service properly
        self.sr_controller_tuner_app_.save_controller(joint_name, self.controller_type, settings, self.file_to_save)


    def refresh_controller_tree_(self, controller_type = "Motor Force"):
        """
        Get the controller settings and their ranges and display them in the tree.
        Buttons and plots will be added unless in edit_only mode.
        Move button will be added if controller is position type
        Buttons "set all" "set selected" and "stop movements" are disabled in edit_only_mode
        Controller settings must exist for every motor of every finger in the yaml file.
        """
        
        if self.sr_controller_tuner_app_.edit_only_mode:
            self._widget.btn_set_selected.setEnabled(False)
            self._widget.btn_set_all.setEnabled(False)
            self._widget.btn_stop_mvts.setEnabled(False)
        else:
            self._widget.btn_set_selected.setEnabled(True)
            self._widget.btn_set_all.setEnabled(True)
            self._widget.btn_stop_mvts.setEnabled(True)
        
        self.controller_type = controller_type
        ctrl_settings = self.sr_controller_tuner_app_.get_controller_settings( controller_type )

        self._widget.tree_ctrl_settings.clear()
        self._widget.tree_ctrl_settings.setColumnCount(ctrl_settings.nb_columns)

        tmp_headers = []
        for header in ctrl_settings.headers:
            tmp_headers.append( header["name"] )
        self._widget.tree_ctrl_settings.setHeaderLabels( tmp_headers )

        hand_item = QTreeWidgetItem(ctrl_settings.hand_item)
        self._widget.tree_ctrl_settings.addTopLevelItem(hand_item)
        for index_finger,finger_settings in enumerate(ctrl_settings.fingers):
            finger_item = QTreeWidgetItem( hand_item, finger_settings )
            self._widget.tree_ctrl_settings.addTopLevelItem(finger_item)
            for motor_settings in ctrl_settings.motors[index_finger]:
                motor_name = motor_settings[1]

                motor_item = QTreeWidgetItem( finger_item, motor_settings )
                self._widget.tree_ctrl_settings.addTopLevelItem(motor_item)

                parameter_values = self.sr_controller_tuner_app_.load_parameters( controller_type, motor_name )
                if parameter_values != -1:
                    #the parameters have been found
                    self.ctrl_widgets[ motor_name ] = {}

                    #buttons for plot/move are not added in edit_only_mode
                    if not self.sr_controller_tuner_app_.edit_only_mode:
                        #add buttons for the automatic procedures (plot / move / ...)
                        frame_buttons = QFrame()
                        layout_buttons = QHBoxLayout()
                        btn_plot = QPushButton("Plot")
                        self.ctrl_widgets[ motor_name ]["btn_plot"] = btn_plot
                        self.ctrl_widgets[ motor_name ]["btn_plot"].clicked.connect(partial(self.on_btn_plot_pressed_, motor_name, self.ctrl_widgets[ motor_name ]["btn_plot"]))
                        layout_buttons.addWidget(btn_plot)
    
                        if self.controller_type in ["Position", "Muscle Position", "Mixed Position/Velocity"]:
                            #only adding Move button for position controllers
                            btn_move = QPushButton("Move")
                            self.ctrl_widgets[ motor_name ]["btn_move"] = btn_move
                            self.ctrl_widgets[ motor_name ]["btn_move"].clicked.connect(partial(self.on_btn_move_pressed_,motor_name, self.ctrl_widgets[ motor_name ]["btn_move"]))
                            layout_buttons.addWidget(btn_move)
                            frame_buttons.setLayout(layout_buttons)
    
                        self._widget.tree_ctrl_settings.setItemWidget(  motor_item, 0, frame_buttons )

                    for index_item,item in enumerate(ctrl_settings.headers):
                        if item["type"] == "Bool":
                            check_box = QCheckBox()

                            if parameter_values["sign"] == 1.0:
                                check_box.setChecked(True)

                            check_box.setToolTip("Check if you want a negative sign\n(if the motor is being driven\n the wrong way around).")
                            self._widget.tree_ctrl_settings.setItemWidget(  motor_item, index_item, check_box )

                            self.ctrl_widgets[ motor_name ]["sign"] = check_box


                        if item["type"] == "Int":
                            spin_box = QSpinBox()
                            spin_box.setRange(int( item["min"] ), int( item["max"] ))

                            param_name = item["name"].lower()
                            spin_box.setValue( int(parameter_values[ param_name ] ) )
                            self.ctrl_widgets[ motor_name ][param_name] = spin_box

                            self._widget.tree_ctrl_settings.setItemWidget(  motor_item, index_item, spin_box )

                        if item["type"] == "Float":
                            spin_box = QDoubleSpinBox()
                            spin_box.setRange( -65535.0, 65535.0 )
                            spin_box.setDecimals(3)

                            param_name = item["name"].lower()
                            spin_box.setValue(float(parameter_values[param_name]))
                            self.ctrl_widgets[ motor_name ][param_name] = spin_box

                            self._widget.tree_ctrl_settings.setItemWidget(  motor_item, index_item, spin_box )

                        motor_item.setExpanded(True)
                else:
                    motor_item.setText(1, "parameters not found - controller tuning disabled")
            finger_item.setExpanded(True)
        hand_item.setExpanded(True)

        for col in range(0, self._widget.tree_ctrl_settings.columnCount()):
            self._widget.tree_ctrl_settings.resizeColumnToContents(col)


    def on_btn_stop_mvts_clicked_(self):
        for move_thread in self.move_threads:
            move_thread.__del__()
        self.move_threads = []


    #########
    #Default methods for the rosgui plugins

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
