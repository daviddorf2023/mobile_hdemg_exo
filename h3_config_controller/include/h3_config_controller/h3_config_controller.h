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

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <controller_interface/controller.h>

#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>


#include "h3_hardware_interface/h3_command_interface.h"
#include "h3_msgs/ControlType.h"
#include "h3_msgs/Joint.h"
#include "h3_msgs/DataRecording.h"
#include "h3_msgs/TriggerOutput.h"


namespace h3_controller_interface
{
  /**
 * @brief This a controller to configure whole robot.
 *
 */

  class H3ConfigController : public controller_interface::Controller<h3_hardware_interface::H3CommandInterface>
  {
  public:
    H3ConfigController() {}
    ~H3ConfigController() {}

    /**
   * Controller functions
   */
    virtual bool init(h3_hardware_interface::H3CommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    virtual void starting(const ros::Time &time);
    virtual void update(const ros::Time &time, const ros::Duration &period);
    virtual void stopping(const ros::Time &time);

    //struct for passing data to realtime update function
    struct ControlTypeCmd
    {
      ControlTypeCmd() {}
      unsigned char control_type[6] = {0};
    };
    struct AssistanceCmd
    {
      AssistanceCmd() {}
      unsigned char assistance[6] = {100, 100, 100, 100, 100, 100};
    };
    struct MinAnglesCmd
    {
      MinAnglesCmd() {}
      double M_PI_R_ = 3.14159265358979323846;
      double M_PI_D_ = 180.00000000000000000000;
      double min_angles[6] = {-100 * M_PI_R_ / M_PI_D_, 0, -25 * M_PI_R_ / M_PI_D_, -100 * M_PI_R_ / M_PI_D_, 0, -25 * M_PI_R_ / M_PI_D_}; // H: -100 K: 0 A: -25;
    };

    struct MaxAnglesCmd
    {
      MaxAnglesCmd() {}
      double M_PI_R_ = 3.14159265358979323846;
      double M_PI_D_ = 180.00000000000000000000;
      double max_angles[6] = {25 * M_PI_R_ / M_PI_D_, 100 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_, 100 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_}; //H: 25, K: 100, A: 25;
    };

    struct TriggerOutputCmd
    {
      TriggerOutputCmd() {}
      int trigger_mode = 0;
    };

    struct RecordingCmd
    {
      RecordingCmd() {}
      std::string base_file_name;
      int start_stop_recording = 0;
      double recording_time_sec = 0.0;
      bool timed_recording = false;
    };

  private:
    const double M_PI_R_ = 3.14159265358979323846;
    const double M_PI_D_ = 180.00000000000000000000;
    int trigger_mode_k_1 = 0;
    ros::Time last_time_;
    unsigned int loop_hz = 100;

    std::vector<double> control_type_ = {0, 0, 0, 0, 0, 0};
    std::vector<double> assistance_ = {100, 100, 100, 100, 100, 100};
    std::vector<double> max_angles_ = {25 * M_PI_R_ / M_PI_D_, 100 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_, 100 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_}; //H: 25, K: 100, A: 25;
    std::vector<double> min_angles_ = {-100 * M_PI_R_ / M_PI_D_, 0, -25 * M_PI_R_ / M_PI_D_, -100 * M_PI_R_ / M_PI_D_, 0, -25 * M_PI_R_ / M_PI_D_}; // H: -100 K: 0 A: -25;

  ControlTypeCmd control_type_cmd_;
  AssistanceCmd assistance_cmd_;
  MinAnglesCmd min_angles_cmd_;
  MaxAnglesCmd max_angles_cmd_;
  TriggerOutputCmd trigger_output_cmd_;
  RecordingCmd recording_cmd_;

  realtime_tools::RealtimeBuffer<ControlTypeCmd> rt_control_type_cmd_;
  realtime_tools::RealtimeBuffer<AssistanceCmd> rt_assistance_cmd_;
  realtime_tools::RealtimeBuffer<MinAnglesCmd> rt_min_angles_cmd_;
  realtime_tools::RealtimeBuffer<MaxAnglesCmd> rt_max_angles_cmd_;
  realtime_tools::RealtimeBuffer<TriggerOutputCmd> rt_trigger_output_cmd_;
  realtime_tools::RealtimeBuffer<RecordingCmd> rt_recording_cmd_;

  h3_hardware_interface::H3CommandHandle robot_handle_;

  /**
   * @brief Service server for change robot parameters
   *
   */
  ros::ServiceServer srv_0;
  ros::ServiceServer srv_1;
  ros::ServiceServer srv_2;
  ros::ServiceServer srv_3;
  ros::ServiceServer srv_4;
  ros::ServiceServer srv_5;

  /**
   * Callback function for service server
   */
  bool setControlType(h3_msgs::ControlType::Request &req,
                      h3_msgs::ControlType::Response &resp);
  bool setMinAngles(h3_msgs::Joint::Request &req, h3_msgs::Joint::Response &resp);
  bool setMaxAngles(h3_msgs::Joint::Request &req, h3_msgs::Joint::Response &resp);
  bool setAssistance(h3_msgs::Joint::Request &req,
                     h3_msgs::Joint::Response &resp);
  bool setDataRecording(h3_msgs::DataRecording::Request &req,
                        h3_msgs::DataRecording::Response &resp);
  bool setTrigger(h3_msgs::TriggerOutput::Request &req,
                  h3_msgs::TriggerOutput::Response &resp);

}; // class

} // namespace h3_controller_interface