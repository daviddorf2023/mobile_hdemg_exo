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

#include <h3_config_controller/h3_config_controller.h>

namespace h3_controller_interface
{
  /// Controller initialization in non-realtime
  bool H3ConfigController::init(h3_hardware_interface::H3CommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    // get named robot from parameter server
    std::string robot_name;
    if (!controller_nh.getParam("robot_name", robot_name))
    {
      ROS_ERROR("robot_name parameter not found in parameter server");
      return false;
    }
    // Get joint configuration from parameter server
    if (!controller_nh.getParam("control_type", control_type_))
      ROS_WARN("control_type parameter not found in parameter server. Value set to default");
    if (!controller_nh.getParam("min_angle", min_angles_))
      ROS_WARN("min_angle parameter not found in parameter server. Value set to default");
    if (!controller_nh.getParam("max_angle", max_angles_))
      ROS_WARN("max_angle parameter not found in parameter server. Value set to default");
    if (!controller_nh.getParam("assistance", assistance_))
      ROS_WARN("assistance parameter not found in parameter server. Value set to default");

    robot_handle_ = hw->getHandle(robot_name);

    // service init
    srv_0 = controller_nh.advertiseService("set_control_type", &H3ConfigController::setControlType, this);
    srv_1 = controller_nh.advertiseService("set_min_angles", &H3ConfigController::setMinAngles, this);
    srv_2 = controller_nh.advertiseService("set_max_angles", &H3ConfigController::setMaxAngles, this);
    srv_3 = controller_nh.advertiseService("set_assistance", &H3ConfigController::setAssistance, this);
    srv_4 = controller_nh.advertiseService("set_data_recording", &H3ConfigController::setDataRecording, this);
    srv_5 = controller_nh.advertiseService("set_trigger_output", &H3ConfigController::setTrigger, this);
    return true;
  }

  /// Controller startup in realtime
  void H3ConfigController::starting(const ros::Time &time)
  {

    ControlTypeCmd control_type;
    MinAnglesCmd min_angles;
    MaxAnglesCmd max_angles;
    AssistanceCmd assistance;

    double joint_position[6] = {0};
    double joint_effort[6] = {0};
   
    for (int i = 0; i < 6; ++i)
    {
      control_type.control_type[i] = control_type_[i];
      min_angles.min_angles[i] = min_angles_[i];
      max_angles.max_angles[i] = max_angles_[i];
      assistance.assistance[i] = assistance_[i];

      if (control_type_[i] == 1 || control_type_[i] == 2)
      {
        joint_position[i] = robot_handle_.getJointAngles()[i];
      }
    }
    robot_handle_.setJointAngles(joint_position);
    robot_handle_.setJointDutyCycles(joint_effort);
    robot_handle_.setJointTorques(joint_effort);

    rt_control_type_cmd_.initRT(control_type);
    rt_assistance_cmd_.initRT(assistance);
    rt_min_angles_cmd_.initRT(min_angles);
    rt_max_angles_cmd_.initRT(max_angles);

    last_time_ = ros::Time::now();
  }

  /// Controller update loop in realtime
  void H3ConfigController::update(const ros::Time &time, const ros::Duration &period)
  {
    if (time - last_time_ >= ros::Duration(1.0 / loop_hz))
    {
      last_time_ = time;
      control_type_cmd_ = *rt_control_type_cmd_.readFromRT();
      assistance_cmd_ = *rt_assistance_cmd_.readFromRT();
      min_angles_cmd_ = *rt_min_angles_cmd_.readFromRT();
      max_angles_cmd_ = *rt_max_angles_cmd_.readFromRT();
      trigger_output_cmd_ = *rt_trigger_output_cmd_.readFromRT();
      recording_cmd_ = *rt_recording_cmd_.readFromRT();

      robot_handle_.setControlType(&control_type_cmd_.control_type[0]);
      robot_handle_.setJointAssistances(&assistance_cmd_.assistance[0]);
      robot_handle_.setMinAngles(&min_angles_cmd_.min_angles[0]);
      robot_handle_.setMaxAngles(&max_angles_cmd_.max_angles[0]);
      robot_handle_.setRecordState(recording_cmd_.base_file_name, recording_cmd_.start_stop_recording, recording_cmd_.recording_time_sec, recording_cmd_.timed_recording);
      /*Trigger logic*/
      if (trigger_output_cmd_.trigger_mode != 1)
      {
        robot_handle_.setTriggerOutput(trigger_output_cmd_.trigger_mode);
        trigger_mode_k_1 = trigger_output_cmd_.trigger_mode;
      }
      else
      {
        if (trigger_output_cmd_.trigger_mode == 1 && (trigger_output_cmd_.trigger_mode != trigger_mode_k_1))
        {
          robot_handle_.setTriggerOutput(trigger_output_cmd_.trigger_mode);
          trigger_mode_k_1 = trigger_output_cmd_.trigger_mode;
        }
        else
        {
          robot_handle_.setTriggerOutput(0);
        }
      }
    }
  }

  /// Controller stopping in realtime
  void H3ConfigController::stopping(const ros::Time &time)
  {
    unsigned char control_type[6] = {0, 0, 0, 0, 0, 0};
    robot_handle_.setControlType(control_type);
  }

  bool H3ConfigController::setControlType(h3_msgs::ControlType::Request &req,
                                          h3_msgs::ControlType::Response &resp)
  {
    ControlTypeCmd cmd;
    cmd.control_type[0] = (unsigned char)req.right_hip;
    cmd.control_type[1] = (unsigned char)req.right_knee;
    cmd.control_type[2] = (unsigned char)req.right_ankle;
    cmd.control_type[3] = (unsigned char)req.left_hip;
    cmd.control_type[4] = (unsigned char)req.left_knee;
    cmd.control_type[5] = (unsigned char)req.left_ankle;
    rt_control_type_cmd_.writeFromNonRT(cmd);
    resp.success = true;
    return true;
  }

  bool H3ConfigController::setMinAngles(h3_msgs::Joint::Request &req,
                                        h3_msgs::Joint::Response &resp)
  {
    MinAnglesCmd cmd;
    cmd.min_angles[0] = req.right_hip;
    cmd.min_angles[1] = req.right_knee;
    cmd.min_angles[2] = req.right_ankle;
    cmd.min_angles[3] = req.left_hip;
    cmd.min_angles[4] = req.left_knee;
    cmd.min_angles[5] = req.left_ankle;
    rt_min_angles_cmd_.writeFromNonRT(cmd);
    resp.success = true;
    return true;
  }
  bool H3ConfigController::setMaxAngles(h3_msgs::Joint::Request &req,
                                        h3_msgs::Joint::Response &resp)
  {
    MaxAnglesCmd cmd;
    cmd.max_angles[0] = req.right_hip;
    cmd.max_angles[1] = req.right_knee;
    cmd.max_angles[2] = req.right_ankle;
    cmd.max_angles[3] = req.left_hip;
    cmd.max_angles[4] = req.left_knee;
    cmd.max_angles[5] = req.left_ankle;
    rt_max_angles_cmd_.writeFromNonRT(cmd);
    resp.success = true;
    return true;
  }
  bool H3ConfigController::setAssistance(h3_msgs::Joint::Request &req,
                                         h3_msgs::Joint::Response &resp)
  {
    AssistanceCmd cmd;
    cmd.assistance[0] = (unsigned char)req.right_hip;
    cmd.assistance[1] = (unsigned char)req.right_knee;
    cmd.assistance[2] = (unsigned char)req.right_ankle;
    cmd.assistance[3] = (unsigned char)req.left_hip;
    cmd.assistance[4] = (unsigned char)req.left_knee;
    cmd.assistance[5] = (unsigned char)req.left_ankle;
    rt_assistance_cmd_.writeFromNonRT(cmd);
    resp.success = true;
    return true;
  }

  bool H3ConfigController::setDataRecording(h3_msgs::DataRecording::Request &req,
                                            h3_msgs::DataRecording::Response &resp)
  {
    RecordingCmd cmd;
    cmd.base_file_name = req.file_base_name;
    cmd.start_stop_recording = req.recording_trigger_mode;
    cmd.recording_time_sec = req.duration_time;
    cmd.timed_recording = req.timed;
    rt_recording_cmd_.writeFromNonRT(cmd);
    resp.success = true;
    return true;
  }

  bool H3ConfigController::setTrigger(h3_msgs::TriggerOutput::Request &req,
                                      h3_msgs::TriggerOutput::Response &resp)
  {
    TriggerOutputCmd trigger_output;
    trigger_output.trigger_mode = req.trigger_output;
    rt_trigger_output_cmd_.writeFromNonRT(trigger_output);
    resp.success = true;
    return true;
  }

  PLUGINLIB_EXPORT_CLASS(h3_controller_interface::H3ConfigController, controller_interface::ControllerBase);
} // namespace h3_controller_interface
