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

#include "h3_hardware_interface/h3_command_interface.h"
#include "h3_msgs/TaskCommand.h"
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <memory>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/UInt8.h>

namespace h3_task_controller
{

    class H3TaskController : public controller_interface::Controller<h3_hardware_interface::H3CommandInterface>
    {
    public:
        H3TaskController() {}
        ~H3TaskController() {}

        //struct to recive command message from ROS subscriber
        struct TaskCommand
        {
            TaskCommand() {}
            unsigned char command;
            unsigned int counter;
        };

        //Controller functions
        virtual bool init(h3_hardware_interface::H3CommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
        virtual void starting(const ros::Time &time);
        virtual void update(const ros::Time &time, const ros::Duration &period);
        virtual void stopping(const ros::Time &time);

        //ROS subscriber callback function
        void setRobotTask(const h3_msgs::TaskCommand::ConstPtr &cmd);

    private:
        ros::Subscriber sub_;
        TaskCommand command_;
        realtime_tools::RealtimeBuffer<TaskCommand> rt_command_;
        h3_hardware_interface::H3CommandHandle robot_handle_;

        unsigned int counter_up_ = 0;
        unsigned int counter_up_last_ = 0;
        double timeout_duration_ = 3;
        bool timeout_ = false;
        unsigned int counter_sub_ = 0;
        ros::Time cmd_emission_time_;
        ros::Time last_time_;
        unsigned char cmd_ant_;

        enum
        {
            S0_ = 0, // initial state
            S1_ = 1, // ROS interface is the command owner
            S2_ = 2, // Bluetooth interface is the command owner
            S3_ = 3, //
            S4_ = 4,
        };
        int present_state_ = 0;
        int next_state_ = 0;
        bool toggle_sound_ = false;
    };
} // namespace h3_task_controller
