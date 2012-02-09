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
#include "../include/sensor_scope/glwidget.hpp"

namespace sensor_scope
{
  const unsigned int GLWidget::nb_buffers_const = 2;

  GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent), current_index(0)
  {
    setMouseTracking(true);

    index_display_list = glGenLists(nb_buffers_const);

    refresh_timer = boost::shared_ptr<QTimer>(new QTimer());
    refresh_timer->setInterval(33);
    connect(refresh_timer.get(), SIGNAL(timeout()), this, SLOT(slot_refresh()));
    refresh_timer->start();
  }

  void GLWidget::initializeGL()
  {
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glColor3f(1,0,0);

    glListBase(index_display_list);
    glCallList( current_index );

    prepare_data();
  }

  void GLWidget::prepare_data()
  {
    glNewList(index_display_list + current_index, GL_COMPILE);

    glBegin(GL_POINTS);
    for( unsigned int i=0; i < 500; ++i)
    {
      if( current_index == 0)
        glVertex2f(i,i);
      else
        glVertex2f(i,i + 200);
    }
    glEnd();
    glEndList();

    current_index += 1;

    if( current_index == nb_buffers_const)
      current_index = 0;
  }

  void GLWidget::slot_refresh()
  {
    update();
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
