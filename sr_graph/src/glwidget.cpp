/**
 * @file   glwidget.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  9 11:17:49 2012
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

#include <QtGui/QMouseEvent>
#include "sr_graph/glwidget.hpp"

namespace sensor_scope
{
  GLWidget::GLWidget(QWidget *parent) : QGLWidget(parent)
  {
    setMouseTracking(true);
  }

  void GLWidget::initializeGL()
  {
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0, 0, 0, 0);
  }

  void GLWidget::resizeGL(int w, int h)
  {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, w, 0, h); // set origin to bottom left corner
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }

  void GLWidget::paintGL()
  {
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1,0,0);
    glBegin(GL_POLYGON);
    glVertex2f(0,0);
    glVertex2f(100,500);
    glVertex2f(500,100);
    glEnd();
  }

  void GLWidget::mousePressEvent(QMouseEvent *event)
  {

  }

  void GLWidget::mouseMoveEvent(QMouseEvent *event)
  {
    //printf("%d, %d\n", event->x(), event->y());
  }

  void GLWidget::keyPressEvent(QKeyEvent* event)
  {
    switch(event->key())
    {
    case Qt::Key_Escape:
      close();
      break;

    default:
      event->ignore();
      break;
    }
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
