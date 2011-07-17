/**
 * @file   data_collector.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Sun Jul 17 14:23:46 2011
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
 * @brief Collects the data to be plotted
 *
 *
 */

#ifndef _DATA_COLLECTOR_HPP_
#define _DATA_COLLECTOR_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <deque>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

namespace data
{
  class DataCollector
  {
  public:
    DataCollector();
    virtual ~DataCollector();

    void msg_callback(const std_msgs::Float64ConstPtr& msg);

    double get_data(int index);
  protected:
    ros::NodeHandle node_tilde_;
    boost::shared_ptr<std::deque<boost::shared_ptr<double> > > data_deque_;

    ros::Subscriber subscriber_;

    boost::mutex mutex;

    static const unsigned int nb_points_to_store_;
  };

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
