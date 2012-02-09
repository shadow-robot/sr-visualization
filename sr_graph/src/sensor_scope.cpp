/**
 * @file   sensor_scope.cpp
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

#include <sr_graph/sensor_scope.hpp>
#include <QtUiTools>

namespace sensor_scope
{
  SensorScope::SensorScope()
  {
    // QUiLoader loader;
    // QFile file(":/ui/test.ui");
    // file.open(QFile::ReadOnly);

    // QWidget* test = loader.load(&file);
    // file.close();

    // test->resize(800, 600);
    // test->show();

    window = boost::shared_ptr<GLWidget>( new GLWidget() );
    window->resize(800,600);
    window->show();
  }

  SensorScope::~SensorScope()
  {}
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
