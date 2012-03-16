/**
 * @file   data_collector.cpp
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
 * @brief Collects the data to be plotted
 *
 *
 */

#include "../include/sensor_scope/data_collector.hpp"

namespace sensor_scope
{
  const unsigned int DataCollector::nb_points_to_store_ = 1e6;

  DataCollector::DataCollector()
    : node_tilde_("~")
  {
    subscriber_ = node_tilde_.subscribe( "/sr_movements/targets", 100, &sensor_scope::DataCollector::msg_callback, this);

    data_deque_ = DoubleDequePtr( new std::deque<boost::shared_ptr<double> >(nb_points_to_store_) );
  }

  DataCollector::~DataCollector()
  {}

  void DataCollector::msg_callback(const std_msgs::Float64ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex);
    data_deque_->push_front(boost::shared_ptr<double>(new double(msg->data)));
    data_deque_->pop_back();
  }

  double DataCollector::get_data(int index)
  {
    boost::mutex::scoped_lock(mutex);

    if( data_deque_->at(index) != 0)
      return *( data_deque_->at(index) );
    else
      return 0;
  }
};

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
