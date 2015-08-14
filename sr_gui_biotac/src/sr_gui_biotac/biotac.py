# Copyright (c) 2013, Shadow Robot Company, SynTouch LLC
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


import rospy
import rospkg
import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtGui import QMessageBox, QWidget, QIcon, QColor, QPainter, QFont
from QtCore import QRectF, QTimer, SIGNAL, SLOT
from sr_robot_msgs.msg import Biotac, BiotacAll
from sr_utilities.hand_finder import HandFinder

class SrGuiBiotac(Plugin):

    def define_electrodes(self): 

        self.sensing_electrodes_x    = rospy.get_param("sr_gui_biotac/sensing_electrodes_x_locations", [6.45,  3.65,  3.65,  6.45,  3.65,  6.45, 0.00, 1.95, -1.95, 0.00, -6.45,- 3.65, -3.65, -6.45, -3.65, -6.45,  0.00,  0.00,  0.00]) ## Physical electrode locations on the sensor
        self.sensing_electrodes_y    = rospy.get_param("sr_gui_biotac/sensing_electrodes_y_locations", [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38,  6.38, 8.38,  7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38, 18.38, 22.18])

        self.excitation_electrodes_x = rospy.get_param("sr_gui_biotac/excitation_electrodes_x_locations", [ 6.45,  3.75,  -3.75,  -6.45])
        self.excitation_electrodes_y = rospy.get_param("sr_gui_biotac/excitation_electrodes_y_locations", [12.48, 24.48,  24.48,  12.48])


        for n in range (len(self.sensing_electrodes_x )) :
            self.sensing_electrodes_x[n] = self.sensing_electrodes_x[n] * self.factor + self.x_display_offset[0]
            self.sensing_electrodes_y[n] = self.sensing_electrodes_y[n] * self.factor + self.y_display_offset[0]


        for n in range (len(self.excitation_electrodes_x ) ) : 
            self.excitation_electrodes_x[n] = self.excitation_electrodes_x[n] * self.factor + self.x_display_offset[0]
            self.excitation_electrodes_y[n] = self.excitation_electrodes_y[n] * self.factor + self.y_display_offset[0]


    def tactile_cb(self, msg):
        self.latest_data = msg

    def get_electrode_colour_from_value (self, value) :
        r = 0.0
        g = 0.0
        b = 255.0

        value = float(value)

        threshold = (0.0,1000.0,2000.0,3000.0,4095.0)

        if value <= threshold[0] :
            pass

        elif value < threshold[1] :

            r = 255
            g = 255 * ( ( value - threshold[0] ) / (threshold[1] - threshold[0]) )
            b = 0 

        elif value < threshold[2] :

            r = 255 * ( ( threshold[2] - value ) / (threshold[2] - threshold[1]) )
            g = 255
            b = 0

        elif value < threshold[3] :

            r = 0
            g = 255
            b = 255 * ( ( value - threshold[2] ) / (threshold[3] - threshold[2]) )

        elif value < threshold[4] :

            r = 0
            g = 255 *( ( threshold[4] - value ) / (threshold[4] - threshold[3]) )
            b = 255


        return QColor(r,g,b)

    def biotac_id_from_dropdown(self) :
        name = self._widget.btSelect.currentText()
        fingers = ["FF","MF", "RF", "LF", "TH"]
        return fingers.index(name)

    def draw_electrode(self, painter, elipse_x, elipse_y, text_x, text_y, colour, text) :

            rect = QRectF(elipse_x, elipse_y, self.RECTANGLE_WIDTH, self.RECTANGLE_HEIGHT)

            painter.setBrush(colour)
            painter.drawEllipse(rect)

            rect.setX(text_x)
            rect.setY(text_y)

            painter.drawText(rect, text)


    def paintEvent(self, paintEvent):
        painter = QPainter(self._widget)
        which_tactile = self.biotac_id_from_dropdown()

        painter.setFont(QFont("Arial", self.label_font_size[0]))

        for n in range(len(self.sensing_electrodes_x)):
            value = self.latest_data.tactiles[which_tactile].electrodes[n]
            eval( "self._widget.lcdE%02d.display(%d)" % (n +1 , value) )
            colour = self.get_electrode_colour_from_value(value)

            elipse_x = self.sensing_electrodes_x[n]
            elipse_y = self.sensing_electrodes_y[n]

            if n < 9 :
                text_x = elipse_x + self.x_display_offset[1]
                text_y = elipse_y + self.y_display_offset[1]

            else :
                text_x = elipse_x + self.x_display_offset[2]
                text_y = elipse_y + self.y_display_offset[2]

            

            self.draw_electrode( painter, elipse_x, elipse_y, text_x, text_y, colour, str(n+1) )

        painter.setFont(QFont("Arial", self.label_font_size[1]))

        for n in range(len(self.excitation_electrodes_x)):
            elipse_x = self.excitation_electrodes_x[n]
            elipse_y = self.excitation_electrodes_y[n]

            colour = QColor(127,127,127)

            text_x = elipse_x + self.x_display_offset[3]
            text_y = elipse_y + self.y_display_offset[3]

            self.draw_electrode( painter, elipse_x, elipse_y, text_x, text_y, colour, "X" + str(n+1) )

            
        self._widget.update()

    def subscribe_to_topic(self, prefix) :
        if prefix:
            rospy.Subscriber(prefix + "tactile", BiotacAll, self.tactile_cb)


    def load_params(self) :
        self.RECTANGLE_WIDTH  = rospy.get_param("sr_gui_biotac/electrode_display_width" , 45)  # Display sizes for electrodes in pixels
        self.RECTANGLE_HEIGHT = rospy.get_param("sr_gui_biotac/electrode_display_height" , 45)
    
        self.factor = rospy.get_param("sr_gui_biotac/display_location_scale_factor" , 17.5) ## Sets the multiplier to go from physical electrode location on the sensor in mm to display location in pixels
        self.x_display_offset = rospy.get_param("sr_gui_biotac/x_display_offset", [150, 12.5, 4.5, 3.5]) # Pixel offsets for displaying electrodes. offset[0] is applied to each electrode. 1,2 and 3 are the label offsets for displaying electrode number. 
        self.y_display_offset = rospy.get_param("sr_gui_biotac/y_display_offset", [-50, 4.0, 4.0, 4.0])
        self.label_font_size = rospy.get_param("sr_gui_biotac/electrode_label_font_sizes", [24, 22]) # Font sizes for labels on sensing + excitation electrodes
        if self._hand_parameters.mapping:
            self.default_topic = \
                '/' + self._hand_parameters.mapping.values()[0] + '/'
        else:
            self.default_topic = ""

    def __init__(self, context):

        super(SrGuiBiotac, self).__init__(context)
        self.setObjectName('SrGuiBiotac')
        self._hand_finder = HandFinder()
        self._hand_parameters = self._hand_finder.get_hand_parameters()
        self.load_params()

        self._publisher = None
        self._widget = QWidget()

        self.latest_data = BiotacAll()

        self.define_electrodes()

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_biotac'), 'uis', 'SrGuiBiotac.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrBiotacUi')

        self.timer = QTimer(self._widget)
        self._widget.connect(self.timer, SIGNAL("timeout()"), self._widget.update)
        self._widget.paintEvent = self.paintEvent

        self.subscribe_to_topic(self.default_topic)
        
        for hand in self._hand_parameters.mapping:
            self._widget.select_prefix.addItem(
                self._hand_parameters.mapping[hand])
        if not self._hand_parameters.mapping:
            rospy.logerr("No hand detected")
            QMessageBox.warning(
                self._widget, "warning", "No hand is detected")
        else:
            self._widget.select_prefix.setCurrentIndex(0)

        self._widget.connect(self._widget.select_prefix, SIGNAL("activated(QString)"), self.subscribe_to_topic)

        self.timer.start(50)


        context.add_widget(self._widget)


