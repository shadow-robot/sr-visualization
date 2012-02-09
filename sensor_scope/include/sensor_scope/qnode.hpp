/**
 * @file   qnode.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  9 11:45:53 2012
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

#ifndef sensor_scope_QNODE_HPP_
#define sensor_scope_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

namespace sensor_scope
{
  class QNode : public QThread
{
    Q_OBJECT
  public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    enum LogLevel
    {
      Debug,
      Info,
      Warn,
      Error,
      Fatal
    };

    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);

  signals:
    void loggingUpdated();
    void rosShutdown();

  private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
    QStringListModel logging_model;
  };
}  // namespace sensor_scope

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* sensor_scope_QNODE_HPP_ */
