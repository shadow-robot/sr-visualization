/**
 * @file   main_window.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  9 11:43:45 2012
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


#ifndef sensor_scope_MAIN_WINDOW_H
#define sensor_scope_MAIN_WINDOW_H

#include <boost/smart_ptr.hpp>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "glwidget.hpp"

#include "../include/sensor_scope/display_item.hpp"

namespace sensor_scope {
  class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

  public slots:
    void on_actionAbout_triggered();

  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    boost::shared_ptr<GLWidget> gl_widget;

    void add_tree_item_();

    std::vector<boost::shared_ptr<DisplayItem> > tree_items_;
  };

}  // namespace sensor_scope

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif // sensor_scope_MAIN_WINDOW_H
