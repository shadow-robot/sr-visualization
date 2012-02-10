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
  const unsigned int GLWidget::nb_buffers_const_ = 2;

  GLWidget::GLWidget(QWidget *parent, QTreeWidget* tree_elements) :
    QGLWidget(parent)
  {
    setMouseTracking(true);

    tree_elements_ = tree_elements;

    add_tree_item_();

    index_display_list_ = glGenLists(nb_buffers_const_);

    data_collector_ = boost::shared_ptr<DataCollector>( new DataCollector() );

    refresh_timer_ = boost::shared_ptr<QTimer>(new QTimer());
    refresh_timer_->setInterval(33);
    connect(refresh_timer_.get(), SIGNAL(timeout()), this, SLOT(slot_refresh()));
    refresh_timer_->start();
  }

  void GLWidget::add_tree_item_()
  {
    QStringList test;
    test << "Test";
    boost::shared_ptr<QTreeWidgetItem> new_item = boost::shared_ptr<QTreeWidgetItem>( new QTreeWidgetItem( tree_elements_, test ) );
    tree_items_.push_back( new_item );
    tree_elements_->addTopLevelItem( new_item.get() );

    for(int col=0; col < tree_elements_->columnCount() ; ++col)
      tree_elements_->resizeColumnToContents(col);
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

    glListBase( index_display_list_ );
    glCallList( 0 );

    prepare_data_();
  }

  void GLWidget::prepare_data_()
  {
    glNewList(index_display_list_ , GL_COMPILE);

    glBegin(GL_POINTS);
    for( int i=0; i < width(); ++i)
    {
      //ROS_ERROR_STREAM(" ["<< i <<"] -> " <<  data_collector_->get_data(i) +  height() / 2 );
      //raw data (we display it unscaled)
      glVertex2f(i, data_collector_->get_data(i) +  height() / 2 );
    }
    glEnd();
    glEndList();
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
