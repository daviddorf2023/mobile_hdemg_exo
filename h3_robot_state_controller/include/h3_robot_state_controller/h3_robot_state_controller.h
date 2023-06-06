/*LGPL-2.1*/
/*
 * Copyright (C) 2021  Technaid S.L. <www.technaid.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:    <support@technaid.com>
 */

#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <pluginlib/class_list_macros.h>
#include "h3_hardware_interface/h3_state_interface.h"
#include <h3_msgs/State.h>
#include <memory>

namespace h3_robot_state_controller
{
  /**
   * @brief Controller for publish H3 states.
   * 
   */
  class H3RobotStateController : public controller_interface::Controller<h3_hardware_interface::H3StateInterface>
  {
  public:
    H3RobotStateController(){};
    ~H3RobotStateController(){};

    bool init(h3_hardware_interface::H3StateInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    void starting(const ros::Time &time);
    void update(const ros::Time &time, const ros::Duration &period);
    void stopping(const ros::Time &time);

  private:
    std::shared_ptr<realtime_tools::RealtimePublisher<h3_msgs::State>> rt_pub_;
    h3_hardware_interface::H3StateInterface *hw_;
    h3_hardware_interface::H3StateHandle robot_handle_;
    ros::Time last_publish_time_;
    double publish_rate_;
  }; // class
} // namespace h3_robot_state_controller