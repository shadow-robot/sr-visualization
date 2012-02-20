/**
 * @file   main_window.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  9 11:49:47 2012
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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/sensor_scope/main_window.hpp"

namespace sensor_scope
{
  using namespace Qt;

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
  {
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    QStringList headers;
    headers << "Motor" << "Joint" << "Type" << "raw value" << "calibrated value";
    ui.tree_options->setHeaderLabels( headers );

    add_tree_item_();

    gl_widget = boost::shared_ptr<GLWidget>( new GLWidget(this) );
    gl_widget->setSizePolicy( QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding) );

    ui.gl_layout->addWidget( gl_widget.get() );

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  }

  MainWindow::~MainWindow() {}

  void MainWindow::add_tree_item_()
  {
    QStringList test;
    test << "Test";
    boost::shared_ptr<DisplayItem> new_item = boost::shared_ptr<DisplayItem>( new DisplayItem( ui.tree_options, 0, "FFJ1" ) );
    tree_items_.push_back( new_item );
    ui.tree_options->addTopLevelItem( new_item.get() );

    for(int col=0; col < ui.tree_options->columnCount() ; ++col)
      ui.tree_options->resizeColumnToContents(col);
  }

  void MainWindow::on_actionAbout_triggered()
  {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
  }


  void MainWindow::ReadSettings()
  {
    QSettings settings("Qt-Ros Package", "sensor_scope");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
  }

  void MainWindow::WriteSettings()
  {
    QSettings settings("Qt-Ros Package", "sensor_scope");
    settings.setValue("test", "test");
  }

  void MainWindow::closeEvent(QCloseEvent *event)
  {
    WriteSettings();
    QMainWindow::closeEvent(event);
  }

}  // namespace sensor_scope


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
