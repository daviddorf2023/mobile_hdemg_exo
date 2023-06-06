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


#include "h3_task_controller/h3_task_controller.h"
#include "h3_hardware_interface/h3_definition.h"

namespace h3_task_controller
{
    bool H3TaskController::init(h3_hardware_interface::H3CommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        // get named robot from parameter server
        std::string robot_name;
        if (!controller_nh.getParam("robot_name", robot_name))
        {
            ROS_ERROR("robot_name parameter no found in parameter server.");
            return false;
        }
        if (!controller_nh.getParam("command_timeout", timeout_duration_))
        {
            ROS_WARN("command_timeout  parameter no found in parameter server. Value set to default");
        }
        // get a handle for command the robot
        robot_handle_ = hw->getHandle(robot_name);

        sub_ = controller_nh.subscribe("command", 1, &H3TaskController::setRobotTask, this);
        return true;
    }

    void H3TaskController::starting(const ros::Time &time)
    {
        last_time_ = ros::Time::now();
        cmd_emission_time_ = last_time_;
    }

    void H3TaskController::update(const ros::Time &time, const ros::Duration &period)
    {
        using namespace h3_definition;

        // Read rt buffer
        TaskCommand task = *rt_command_.readFromRT();
        counter_up_ = task.counter;

        // Detect Timeout
        if (time - last_time_ > ros::Duration(timeout_duration_))
        {
            if ((counter_up_ - counter_up_last_) == 0)
            {
                timeout_ = true;
            }
            last_time_ = time;
            counter_up_last_ = counter_up_;
        }

        // Read hardware interface
        unsigned char exo_state = robot_handle_.getH3State();
        unsigned char command_app = robot_handle_.getH3CommandApp();
        unsigned char connection_status = robot_handle_.getExoConnectionStatus();
        double *joint_position = robot_handle_.getJointAngles();

        if (present_state_ == S0_ && command_app == 0)
        {
            next_state_ = S3_;
        }
        if (present_state_ == S0_ && command_app == 1)
        {
            next_state_ = S2_;
        }
        if (present_state_ == S1_ && (timeout_ || connection_status == 0 || task.command == 52) && (exo_state >= WalkSpeed1 && exo_state <= WalkSpeed10))
        {
            next_state_ = S3_;
        }
        else if (present_state_ == S1_ && task.command == 52 && !(exo_state >= WalkSpeed1 && exo_state <= WalkSpeed10))
        {
            next_state_ = S2_;
        }
        if (present_state_ == S2_ && task.command == 51 && (exo_state >= WalkSpeed1 && exo_state <= WalkSpeed10))
        {
            next_state_ = S4_;
        }
        if (present_state_ == S2_ && task.command == 51 && !(exo_state >= WalkSpeed1 && exo_state <= WalkSpeed10))
        {
            next_state_ = S1_;
        }
        if (present_state_ == S3_ && !(exo_state == IsStopppingWalk) && !(exo_state >= WalkSpeed1 && exo_state <= WalkSpeed10))
        {
            next_state_ = S2_;
        }
        if (present_state_ == S4_ && !(exo_state == IsStopppingWalk) && !(exo_state >= WalkSpeed1 && exo_state <= WalkSpeed10))
        {
            next_state_ = S1_;
        }
        present_state_ = next_state_;

        switch (present_state_)
        {
        case S0_:
            break;
        case S1_:
            if (task.command != cmd_ant_ && task.command >= 0 && task.command <= 51)
            {
                robot_handle_.setRobotTask(task.command);
                cmd_ant_ = task.command;
                cmd_emission_time_ = time;
            }
            else
            {
                if (time - cmd_emission_time_ > ros::Duration(0.200))
                {
                    robot_handle_.setRobotTask(60);
                }
            }
            if (!(exo_state >= WalkSpeed1 && exo_state <= WalkSpeed10))
            {
                timeout_ = false;
            }
            break;
        case S2_:
            robot_handle_.setRobotTask(52);
            break;
        case S3_:
            robot_handle_.setRobotTask(StopGait);
            break;
        case S4_:
            robot_handle_.setRobotTask(StopGait);
            break;
        default:
            break;
        }
    }

    void H3TaskController::stopping(const ros::Time &time)
    {
        if (robot_handle_.getH3State())
        {
            robot_handle_.setRobotTask(0);
        }
    }

    void H3TaskController::setRobotTask(const h3_msgs::TaskCommand::ConstPtr &cmd)
    {
        TaskCommand task;
        task.command = cmd->command;
        counter_sub_++;
        if (counter_sub_ > 1000000)
        {
            counter_sub_ = 0;
        }
        task.counter = counter_sub_;
        rt_command_.writeFromNonRT(task);
    }
    PLUGINLIB_EXPORT_CLASS(h3_task_controller::H3TaskController, controller_interface::ControllerBase);
} // namespace h3_task_controller