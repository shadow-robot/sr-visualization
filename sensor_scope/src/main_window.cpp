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

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    if ( ui.checkbox_remember_settings->isChecked() )
    {
      on_button_connect_clicked(true);
    }
  }

  MainWindow::~MainWindow() {}


  void MainWindow::showNoMasterMessage()
  {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
  }

  void MainWindow::on_button_connect_clicked(bool check )
  {
    if ( ui.checkbox_use_environment->isChecked() )
    {
      if ( !qnode.init() )
      {
        showNoMasterMessage();
      }
      else
      {
        ui.button_connect->setEnabled(false);
      }
    }
    else
    {
      if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                        ui.line_edit_host->text().toStdString()) )
      {
        showNoMasterMessage();
      }
      else
      {
        ui.button_connect->setEnabled(false);
        ui.line_edit_master->setReadOnly(true);
        ui.line_edit_host->setReadOnly(true);
        ui.line_edit_topic->setReadOnly(true);
      }
    }
  }


  void MainWindow::on_checkbox_use_environment_stateChanged(int state)
  {
    bool enabled;
    if ( state == 0 )
    {
      enabled = true;
    }
    else
    {
      enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
  }

  void MainWindow::updateLoggingView()
  {
    ui.view_logging->scrollToBottom();
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
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked )
    {
      ui.line_edit_master->setEnabled(false);
      ui.line_edit_host->setEnabled(false);
      //ui.line_edit_topic->setEnabled(false);
    }
  }

  void MainWindow::WriteSettings()
  {
    QSettings settings("Qt-Ros Package", "sensor_scope");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

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
