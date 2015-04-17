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
from QtCore import QRectF, QTimer, SIGNAL
from sr_robot_msgs.msg import Biotac, BiotacAll

NUMBER_OF_SENSING_ELECTRODES    = 19
NUMBER_OF_EXCITATION_ELECTRODES = 4

RECTANGLE_WIDTH  = 45
RECTANGLE_HEIGHT = 45
    
factor = 17.5

x_offset_1 = 150
x_offset_2 = 12.5
x_offset_3 = 4.5
x_offset_4 =  3.5

y_offset_1 = -50
y_offset_2 =  4.0
y_offset_3 =  4.0
y_offset_4 =  4.0

font_size_1 = 24
font_size_2 = 22



class SrGuiBiotac(Plugin):
    """
    A rosgui plugin for visualising biotac sensord data
    """

    def define_electrodes(self): 

        self.sensing_electrodes_x    = [6.45,  3.65,  3.65,  6.45,  3.65,  6.45, 0.00, 1.95, -1.95, 0.00, -6.45,- 3.65, -3.65, -6.45, -3.65, -6.45,  0.00,  0.00,  0.00]
        self.sensing_electrodes_y    = [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38,  6.38, 8.38,  7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38, 18.38, 22.18]

        self.excitation_electrodes_x = [ 6.45,  3.75,  -3.75,  -6.45]
        self.excitation_electrodes_y = [12.48, 24.48,  24.48,  12.48]


        for n in range (NUMBER_OF_SENSING_ELECTRODES) :
            self.sensing_electrodes_x[n] = self.sensing_electrodes_x[n] * factor + x_offset_1
            self.sensing_electrodes_y[n] = self.sensing_electrodes_y[n] * factor + y_offset_1


        for n in range (NUMBER_OF_EXCITATION_ELECTRODES) :
            self.excitation_electrodes_x[n] = self.excitation_electrodes_x[n] * factor + x_offset_1
            self.excitation_electrodes_y[n] = self.excitation_electrodes_y[n] * factor + y_offset_1


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

    def paintEvent(self, paintEvent):
        painter = QPainter(self._widget)

        which_tactile = 0

        self.define_electrodes()
        
        for n in range(NUMBER_OF_SENSING_ELECTRODES) :
            value = self.latest_data.tactiles[which_tactile].electrodes[n]
            eval( "self._widget.lcdE%02d.display(%d)" % (n +1 , value) )
            colour = self.get_electrode_colour_from_value(value)

            rect = QRectF(self.sensing_electrodes_x[n], self.sensing_electrodes_y[n], RECTANGLE_WIDTH, RECTANGLE_HEIGHT)

            painter.setBrush(colour)
#            painter.setFont(QFont("Arial", font_size_1))
            painter.drawEllipse(rect)

            """
            if n < 9 :
                self.sensing_electrodes[n].setX (self.sensing_electrodes[n].x() + x_offset_2)
                self.sensing_electrodes[n].setY (self.sensing_electrodes[n].y() + y_offset_2)
            else :
                self.sensing_electrodes[n].setX (self.sensing_electrodes[n].x() + x_offset_3)
                self.sensing_electrodes[n].setY (self.sensing_electrodes[n].y() + y_offset_3)
             """

            
        self._widget.update()

    def __init__(self, context):

        super(SrGuiBiotac, self).__init__(context)
        self.setObjectName('SrGuiBiotac')


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

        rospy.Subscriber("tactile", BiotacAll, self.tactile_cb)

        self.timer.start(50)


        context.add_widget(self._widget)


