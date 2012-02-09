/**
 * @file   sensor_scope.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  9 10:59:55 2012
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

#ifndef _SENSOR_SCOPE_HPP_
#define _SENSOR_SCOPE_HPP_

#include <boost/smart_ptr.hpp>
#include "sr_graph/glwidget.hpp"

namespace sensor_scope
{
  class SensorScope
  {
  public:
    SensorScope();
    virtual ~SensorScope();

  protected:
    boost::shared_ptr<GLWidget> window;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
