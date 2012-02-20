/**
 * @file   display_item.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Feb 10 15:03:55 2012
 *
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 *
 * @brief
 *
 *
 */

#include "../include/sensor_scope/display_item.hpp"

#include <sstream>

namespace sensor_scope
{
  DisplayItem::DisplayItem(QTreeWidget* parent, int motor_id, std::string joint_name) :
    QTreeWidgetItem(parent, QTreeWidgetItem::Type)
  {
    std::stringstream ss;
    ss << "Motor " << motor_id;
    setText(0, ss.str().c_str());
    setText(1, joint_name.c_str());
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
