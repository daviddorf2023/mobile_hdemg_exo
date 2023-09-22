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

#include "h3_robot_state_controller/h3_robot_state_controller.h"

namespace h3_robot_state_controller
{

  bool H3RobotStateController::init(h3_hardware_interface::H3StateInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {

    if (!controller_nh.getParam("publish_rate", publish_rate_))
    {
      ROS_ERROR("Publish_rate parameter no found in parameter server.");
      return false;
    }
    std::string robot_name;
    if (!controller_nh.getParam("robot_name", robot_name))
    {
      ROS_ERROR("Robot parameter no found in parameter server.");
      return false;
    }
    // Get a robot handle
    robot_handle_ = hw->getHandle(robot_name);

    rt_pub_.reset(new realtime_tools::RealtimePublisher<h3_msgs::State>(root_nh, "robot_states", 4));

    // Get joint names
    rt_pub_->msg_.joint_name = robot_handle_.getJointNames();

    // Initialized storage
    for (unsigned i = 0; i < 6; i++)
    {
      rt_pub_->msg_.joint_position.push_back(0.0);
      rt_pub_->msg_.joint_velocity.push_back(0.0);
      rt_pub_->msg_.joint_torque_sensor.push_back(0.0);
      rt_pub_->msg_.joint_motor_torque.push_back(0.0);
      rt_pub_->msg_.joint_control_type.push_back(0.0);
    }
    rt_pub_->msg_.right_heel_sensor = 0.0;
    rt_pub_->msg_.right_toe_sensor = 0.0;
    rt_pub_->msg_.left_heel_sensor = 0.0;
    rt_pub_->msg_.left_toe_sensor = 0.0;
    rt_pub_->msg_.battery_voltage = 0.0;
    rt_pub_->msg_.trigger_counter = 0.0;
    rt_pub_->msg_.status = 0.0;
    rt_pub_->msg_.command_app = 0.0;
    rt_pub_->msg_.trigger_counter = 0.0;
    rt_pub_->msg_.runtime_min = 0.0;
    rt_pub_->msg_.runtime_sec = 0.0;
    rt_pub_->msg_.runtime_csec = 0.0;
    rt_pub_->msg_.connection_status = 0.0;
    rt_pub_->msg_.recording_status = 0.0;

    return true;
  }

  void H3RobotStateController::starting(const ros::Time &time)
  {
    // Initialize time
    last_publish_time_ = time;
  }

  void H3RobotStateController::update(const ros::Time &time, const ros::Duration &period)
  {
    // Publish ROS message
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
    {
      if (rt_pub_->trylock())
      {
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

        rt_pub_->msg_.header.stamp = time;
        rt_pub_->msg_.name = robot_handle_.getName();

        unsigned int number_of_joints = robot_handle_.getNumberOfJoints();
        for (int i = 0; i < number_of_joints; i++)
        {
          if (robot_handle_.getExoControlType())
            rt_pub_->msg_.joint_control_type[i] = robot_handle_.getExoControlType()[i];
          if (robot_handle_.getJointAngles())
            rt_pub_->msg_.joint_position[i] = robot_handle_.getJointAngles()[i];
          if (robot_handle_.getJointTorques())
            rt_pub_->msg_.joint_torque_sensor[i] = robot_handle_.getJointTorques()[i];
          if (robot_handle_.getJointVelocities())
            rt_pub_->msg_.joint_velocity[i] = robot_handle_.getJointVelocities()[i];
          if (robot_handle_.getJointMotorTorques())
            rt_pub_->msg_.joint_motor_torque[i] = robot_handle_.getJointMotorTorques()[i];
          if (robot_handle_.getExoControlType())
            rt_pub_->msg_.joint_control_type[i] = robot_handle_.getExoControlType()[i];
        }
        rt_pub_->msg_.right_heel_sensor = robot_handle_.getRightHeelSensor();
        rt_pub_->msg_.right_toe_sensor = robot_handle_.getRightToeSensor();
        rt_pub_->msg_.left_heel_sensor = robot_handle_.getLeftHeelSensor();
        rt_pub_->msg_.left_toe_sensor = robot_handle_.getLeftToeSensor();
        rt_pub_->msg_.battery_voltage = robot_handle_.getBatteryVoltage();
        rt_pub_->msg_.trigger_input = robot_handle_.getTriggerInput();
        rt_pub_->msg_.status = robot_handle_.getH3State();
        rt_pub_->msg_.command_app = robot_handle_.getH3CommandApp();
        rt_pub_->msg_.trigger_counter = robot_handle_.getTriggerCounter();
        rt_pub_->msg_.runtime_min = robot_handle_.getH3RuntimeMin();
        rt_pub_->msg_.runtime_sec = robot_handle_.getH3RuntimeSec();
        rt_pub_->msg_.runtime_csec = robot_handle_.getH3RuntimeCSec();
        rt_pub_->msg_.connection_status = robot_handle_.getExoConnectionStatus();
        rt_pub_->msg_.recording_status = robot_handle_.getRecordingStatus();
      }
      rt_pub_->unlockAndPublish();
    }
  }

  void H3RobotStateController::stopping(const ros::Time &time)
  {
  }
  // Register plugin
  PLUGINLIB_EXPORT_CLASS(h3_robot_state_controller::H3RobotStateController, controller_interface::ControllerBase);

} // namespace h3_robot_state_controller