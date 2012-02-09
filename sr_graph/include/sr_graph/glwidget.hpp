/**
 * @file   glwidget.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  9 11:15:24 2012
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

#ifndef _GL_WIDGET_HPP_
#define _GL_WIDGET_HPP_

#include <QtOpenGL/QGLWidget>
#include "sr_graph/data_collector.hpp"

namespace sensor_scope
{
  class GLWidget : public QGLWidget {
    Q_OBJECT // must include this if you use Qt signals/slots

  public:
    GLWidget(QWidget *parent = NULL);

  protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

    boost::shared_ptr<DataCollector> data_collector;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
