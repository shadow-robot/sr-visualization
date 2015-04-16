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


import rospy
import rospkg
import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtGui import QMessageBox, QWidget, QIcon, QRect
from sr_robot_msgs.msg import Biotac, BiotacAll

class SrGuiBiotac(Plugin):
    """
    A rosgui plugin for visualising biotac sensord data
    """
    NUMBER_OF_SENSING_ELECTRODES    = 19
    NUMBER_OF_EXCITATION_ELECTRODES = 4

    RECTANGLE_WIDTH  = 45
    RECTANGLE_HEIGHT = 45
    

    def define_electrodes(self): 
        self.sensing_electrodes    = []
        self.excitation_electrodes = []

        ## My sincerest appologies for the rest of this function. This is how works in the code I'm copying from, and I don't have time/energy to make it better (it's already better than it was) :'(
        ## There best have been a really f*cking good reason why it was done like this in the first place.


        factor = 17.5

        x_offset_1 = 150
        x_offset_2 = x_offset_1 + 12.5
        x_offset_3 = x_offset_1 +  4.5
        x_offset_4 = x_offset_1 +  3.5
  
        y_offset_1 = -50
        y_offset_2 = y_offset_1 +  4.0
        y_offset_3 = y_offset_1 +  4.0
        y_offset_4 = y_offset_1 +  4.0


        self.sensing_electrodes.append(QRect( 7.58,  6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(11.28,  3.65, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(14.78,  3.65, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(16.58,  6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(19.08,  3.65, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(21.98,  6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect( 4.38,  0.00, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect( 6.38,  1.95, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect( 6.38, -1.95, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect( 8.38,  0.00, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect( 7.58, -6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(11.28, -3.65, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(14.78, -3.65, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(16.58, -6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(19.08, -3.65, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(21.98, -6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(11.38,  0.00, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(18.38,  0.00, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.sensing_electrodes.append(QRect(22.18,  0.00, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))

        self.excitation_electrodes.append(QRect(12.48,  6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.excitation_electrodes.append(QRect(24.48,  3.75, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.excitation_electrodes.append(QRect(24.48, -3.75, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))
        self.excitation_electrodes.append(QRect(12.48, -6.45, RECTANGLE_WIDTH, RECTANGLE_HEIGHT))

        ## K, here's where it get's really good :-/
    
        for n in range (NUMBER_OF_SENSING_ELECTRODES) :
            old_x = self.sensing_electrodes[n].x()
            old_y = self.sensing_electrodes[n].y()

            if n < 9 :
                self.sensing_electrodes[n].setX(old_y * factor + x_offset_2)
                self.sensing_electrodes[n].setY(old_x * factor + y_offset_2)
            else :
                self.sensing_electrodes[n].setX(old_y * factor + x_offset_3)
                self.sensing_electrodes[n].setY(old_x * factor + y_offset_3)

        for n in range (NUMBER_OF_EXCITATION_ELECTRODES) :
            old_x = self.excitation_electrodes[n].x()
            old_y = self.excitation_electrodes[n].y()

            self.sensing_electrodes[n].setX(old_y * factor + x_offset_1)
            self.sensing_electrodes[n].setY(old_x * factor + y_offset_1)



self.sensing_electrodes[x]
                              
                           


    def __init__(self, context):
        super(SrGuiBiotac, self).__init__(context)
        self.setObjectName('SrGuiBiotac')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('sr_gui_biotac'), 'uis', 'SrGuiBiotac.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrBiotacUi')
        context.add_widget(self._widget)
