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

#include "h3_hardware_interface/h3_hardware_interface.h"

namespace h3_hardware_interface
{

  H3Hardware::H3Hardware(ros::NodeHandle &nh) : nh_(nh)
  {
  }

  H3Hardware::~H3Hardware()
  {
    h3_handle.Stop();
  }

  bool H3Hardware::initialize()
  {

    if (nh_.getParam("CAN_port", CAN_port_))
    {
      ROS_INFO("CAN port retrieved from parameter server %d", CAN_port_);
    }
    else
    {
      ROS_WARN("Can't retrieve CAN port from parameter server, value set to defualt: %d", CAN_port_);
    }

    ROS_INFO("Starting communication with port: %d", CAN_port_);

    if (nh_.getParam("recording_convention", recording_convention_))
    {
      ROS_INFO("Recording convention retrieved from parameter server");
    }
    else
    {
      ROS_WARN("Can't retrieve recording convention from parameter server, value set to defualt");
    }
    if (nh_.getParam("recording_mode", recording_mode_))
    {
      ROS_INFO("Recording mode retrieved from parameter server");
    }
    else
    {
      ROS_WARN("Can't recording mode from parameter server, value set to default");
    }
    if (nh_.getParam("joints", joint_names_))
    {
      ROS_INFO("Joint names retrieved from parameter server");
    }
    else
    {
      ROS_WARN("Don't can retrieved joint names from parameter server");
    }
    if (nh_.getParam("robot_name", robot_name_))
    {
      ROS_INFO("Robot name retrieved from parameter server");
    }
    else
    {
      ROS_WARN("Don't can retrieved robot name from parameter server, names set to default");
    }
    if (nh_.getParam("external_CAN_mode", external_CAN_mode_))
    {
      ROS_INFO("CAN communication mode retrieved from parameter server");
    }
    else
    {
      ROS_WARN("Don't can retrieved CAN communication mode from parameter server, communication mode set to default");
    }

    if (nh_.getParam("modify_internal_controllers", modify_internal_controllers_))
      ;
    if (modify_internal_controllers_)
    {
      /* Position Controllers*/
      if (nh_.getParam("position_kp", position_kp_))
        ROS_INFO("Position controller kp parameters retrieved from parameter server");
      else
        ROS_WARN("Don't can retrieved position controller kp parameters from parameter server, kp constats set to default");
      if (nh_.getParam("position_ki", position_ki_))
        ROS_INFO("Position controller ki parameters retrieved from parameter server");
      else
        ROS_WARN("Don't can retrieved position controller ki parameters from parameter server, ki constats set to default");
      if (nh_.getParam("position_kd", position_kd_))
        ROS_INFO("Position controller kd parameters retrieved from parameter server");
      else
        ROS_WARN("Don't can retrieved position controller kd parameters from parameter server, kd constats set to default");
      if (nh_.getParam("position_i_clamp", position_i_clamp_))
        ROS_INFO("Position controller i_clamp parameters retrieved from parameter server");
      else
        ROS_WARN("Unable to retrieve i_clamp parameters from torque controller from parameter server, i_clamp parameters are set by default");

      /* Stiffness Controller */
      if (nh_.getParam("stiffness_kp", stiffness_kp_))
        ROS_INFO("Stiffness controller kp parameters retrieved from parameter server");
      else
        ROS_WARN("Don't can retrieved stiffness controller kp parameters from parameter server, kp constats set to default");
      if (nh_.getParam("stiffness_ki", stiffness_ki_))
        ROS_INFO("Stiffness controller ki parameters retrieved from parameter server");
      else
        ROS_WARN("Don't can retrieved stiffness controller ki parameters from parameter server, ki constats set to default");
      if (nh_.getParam("stiffness_kd", stiffness_kd_))
        ROS_INFO("Stiffness controller kd parameters retrieved from parameter server");
      else
        ROS_WARN("Don't can retrieved stiffness controller kd parameters from parameter server, kd constats set to default");
      if (nh_.getParam("stiffness_i_clamp", stiffness_i_clamp_))
        ROS_INFO("Stiffness controller i_clamp parameters retrieved from parameter server");
      else
        ROS_WARN("Unable to retrieve i_clamp parameters from torque controller from parameter server, i_clamp parameters are set by default");

      /* Torque Controller */
      if (nh_.getParam("torque_kp", torque_kp_))
        ROS_INFO("Torque controller kp parameters retrieved from parameter server");
      else
        ROS_WARN("Don't can retrieved torque controller kp parameters from parameter server, kp constats set to default");
      if (nh_.getParam("torque_ki", torque_ki_))
        ROS_INFO("Torque controller ki parameters retrieved from parameter server");
      else
        ROS_WARN("Unable to retrieve i_clamp parameters from torque controller from parameter server, i_clamp parameters are set by default");
      if (nh_.getParam("torque_kd", torque_kd_))
        ROS_INFO("Torque controller kd parameters retrieved from parameter server");
      else
        ROS_WARN("Unable to retrieve i_clamp parameters from torque controller from parameter server, i_clamp parameters are set by default");
      if (nh_.getParam("torque_i_clamp", torque_i_clamp_))
        ROS_INFO("Torque controller i_clamp parameters retrieved from parameter server");
      else
        ROS_WARN("Unable to retrieve i_clamp parameters from torque controller from parameter server, i_clamp parameters are set by default");
    }

    /**
     * @brief Joint Limits
     * 
     */
    joint_limits_.resize(joint_names_.size());
    for (int i = 0; i < joint_limits_.size(); ++i)
    {
      if (joint_limits_interface::getJointLimits(joint_names_[i], nh_, joint_limits_[i]))
      {
        ROS_INFO("Limits retrived from parameter server for joint: %s", joint_names_[i].c_str());
      }
      else
      {
        ROS_WARN("Unable to retrive limits from parameter server for joint: %s", joint_names_[i].c_str());
      }
    }

    /**
     * @brief Handle standard interfaces.
     *
     */
    for (int i = 0; i < joint_names_.size(); ++i)
    {
      // Joints State Interface
      hardware_interface::JointStateHandle joint_state_handle(joint_names_[i], &joint_angles_[i], &joint_velocities_[i], &joint_motor_torques_[i]);
      joint_state_interface_.registerHandle(joint_state_handle);

      // Position Joint Interface: Position based-joints.
      hardware_interface::JointHandle position_joint_handle(joint_state_handle, &position_setpoint_[i]);
      position_joint_interface_.registerHandle(position_joint_handle);

      // Joints Effort Interface: Effort based joints (strain gauge joint torque and Open loop).
      hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &effort_command_[i]);
      effort_joint_interface_.registerHandle(effort_joint_handle);

      //Stiffness Joint Interface: Stiffness based joints
      hardware_interface::StiffnessJointHandle stiffness_joint_handle(joint_state_handle, &position_setpoint_[i], &stiffness_setpoint_[i]);
      stiffness_joint_interface_.registerHandle(stiffness_joint_handle);

      //Joint Limits Interface: Position
      joint_limits_interface::PositionJointSaturationHandle position_joint_limit_handle(position_joint_handle, joint_limits_[i]);
      position_joint_limit_interface_.registerHandle(position_joint_limit_handle);

      //Joint Limits Interface: Effort base joints (strain gauge joint torque and Open loop)
      joint_limits_interface::EffortJointSaturationHandle effort_joint_limit_handle(effort_joint_handle, joint_limits_[i]);
      effort_joint_limit_interface_.registerHandle(effort_joint_limit_handle);
    }

    /**
     * @brief Register the standard interfaces supported by EXO-H3.
     *
     */
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);

    // Register Exo-H3 Specific State Interface.
    registerInterface(&stiffness_joint_interface_);

    //h3_state_.robot_name = robot_name_;
    h3_state_.number_of_joints = number_of_joints_;
    h3_state_.joint_angles = &joint_angles_[0];
    h3_state_.joint_velocities = &joint_velocities_[0];
    h3_state_.joint_torques = &joint_torques_[0];
    h3_state_.joint_motor_torques = &joint_motor_torques_[0];
    h3_state_.right_heel_sensor = &right_heel_sensor_;
    h3_state_.right_toe_sensor = &right_toe_sensor_;
    h3_state_.left_heel_sensor = &left_heel_sensor_;
    h3_state_.left_toe_sensor = &left_toe_sensor_;
    h3_state_.battery_voltage = &battery_voltage_;
    h3_state_.h3_state = &h3_state_TC_;
    h3_state_.trigger_input = &trigger_input_;
    h3_state_.h3_commandApp = &h3_commandApp_;
    h3_state_.trigger_counter = &trigger_counter_;
    h3_state_.h3_runtime_min = &h3_runtime_min_;
    h3_state_.h3_runtime_sec = &h3_runtime_sec_;
    h3_state_.h3_runtime_csec = &h3_runtime_csec_;
    h3_state_.h3_connection_status = &h3_connection_status_;
    h3_state_.control_type = &control_type_status_[0];
    h3_state_.recording_status = &record_status_;

    h3_hardware_interface::H3StateHandle h3_state_handle(h3_state_, robot_name_, joint_names_);
    h3_state_interface_.registerHandle(h3_state_handle);
    registerInterface(&h3_state_interface_);

    // Register Exo-H3 Specific Command Interface.
    h3_command_.control_type = &control_type_[0];
    h3_command_.position_setpoint = &position_setpoint_[0];
    h3_command_.effort_command = &effort_command_[0];
    h3_command_.stiffness_setpoint = &stiffness_setpoint_[0];
    h3_command_.robot_tasK = &robot_task_;
    h3_command_.percentage_assistance = &percentage_assistance_[0];
    h3_command_.min_angles = &min_angles_[0];
    h3_command_.max_angles = &max_angles_[0];
    h3_command_.trigger_output = &trigger_output_;

    h3_command_.file_name = &base_file_name_;
    h3_command_.start_stop_recording = &start_stop_recording_;
    h3_command_.recording_time_sec = &recording_time_sec_;
    h3_command_.timed_recording = &timed_recording_;

    h3_hardware_interface::H3CommandHandle h3_h(h3_state_handle, h3_command_);
    h3_command_interface_.registerHandle(h3_h);
    registerInterface(&h3_command_interface_);

    //Start CAN communication
    int status_code;
    if (external_CAN_mode_ == 3)
    {
      status_code = h3_handle.CANBegin(CAN_port_, h3_definition::StatusCANFrame::ExoStateFrame2, h3_definition::CANTransmissionHighResolution);
    }
    else
    {
      status_code = h3_handle.CANBegin(CAN_port_, h3_definition::StatusCANFrame::ExoStateFrame, h3_definition::CANTransmissionLowResolution);
    }
    h3_handle.getCANErrorText(status_code, 9, status_text_);

    // Send parameters to internal controllers
    if (modify_internal_controllers_)
    {
      /* Position Controller */
      if (h3_handle.writeControllersGains(h3_definition::SelectControlType::PositionControl, position_kp_, position_ki_, position_kd_, position_i_clamp_))
        ROS_INFO("Position controller parameters write successful");
      else
        ROS_WARN("Unable to write position controller parameters");

      /* Stiffness Controller */
      if (h3_handle.writeControllersGains(h3_definition::SelectControlType::StiffnessControl, stiffness_kp_, stiffness_ki_, stiffness_kd_, stiffness_i_clamp_))
        ROS_INFO("Stiffness controller gains write successful");
      else
        ROS_WARN("Unable to write stiffness controller parameters");

      /* Torque Controller*/
      if (h3_handle.writeControllersGains(h3_definition::SelectControlType::TorqueControl, torque_kp_, torque_ki_, torque_kd_, torque_i_clamp_))
        ROS_INFO("Torque controller parameters write successful");
      else
        ROS_WARN("Unable to write torque controller parameters");

      /* Check */
      std::vector<double> kp;
      std::vector<double> ki;
      std::vector<double> kd;
      std::vector<double> i_clamp;
      int check_counter = 0;
      if (h3_handle.readControllersGains(h3_definition::SelectControlType::PositionControl, kp, ki, kd, i_clamp))
      {
        for (int i = 0; i < joint_names_.size(); ++i)
        {
          if (position_kp_[i] - kp[i] < 0.1 && position_kp_[i] - kp[i] > -0.1)
            check_counter++;
          if (position_ki_[i] - ki[i] < 0.1 && position_ki_[i] - ki[i] > -0.1)
            check_counter++;
          if (position_kd_[i] - kd[i] < 0.1 && position_kd_[i] - kd[i] > -0.1)
            check_counter++;
          if (position_i_clamp_[i] - i_clamp[i] < 0.1 && position_i_clamp_[i] - i_clamp[i] > -0.1)
            check_counter++;
        }
      }
      if (check_counter == (4 * joint_names_.size()))
        ROS_INFO("Position controller gains check successful");
      else
        ROS_WARN("Position controller gains check failed");

      kp.clear();
      ki.clear();
      kd.clear();
      i_clamp.clear();
      check_counter = 0;
      if (h3_handle.readControllersGains(h3_definition::SelectControlType::StiffnessControl, kp, ki, kd, i_clamp))
      {
        for (int i = 0; i < joint_names_.size(); ++i)
        {
          if (stiffness_kp_[i] - kp[i] < 0.1 && stiffness_kp_[i] - kp[i] > -0.1)
            check_counter++;
          if (stiffness_ki_[i] - ki[i] < 0.1 && stiffness_ki_[i] - ki[i] > -0.1)
            check_counter++;
          if (stiffness_kd_[i] - kd[i] < 0.1 && stiffness_kd_[i] - kd[i] > -0.1)
            check_counter++;
          if (stiffness_i_clamp_[i] - i_clamp[i] < 0.1 && stiffness_i_clamp_[i] - i_clamp[i] > -0.1)
            check_counter++;
        }
      }
      if (check_counter == (4 * joint_names_.size()))
        ROS_INFO("Stiffness controller gains check successful");
      else
        ROS_WARN("Stiffness controller gains check failed");

      kp.clear();
      ki.clear();
      kd.clear();
      i_clamp.clear();
      check_counter = 0;
      if (h3_handle.readControllersGains(h3_definition::SelectControlType::TorqueControl, kp, ki, kd, i_clamp))
      {
        for (int i = 0; i < joint_names_.size(); ++i)
        {
          if (torque_kp_[i] - kp[i] < 0.1 && torque_kp_[i] - kp[i] > -0.1)
            check_counter++;
          if (torque_ki_[i] - ki[i] < 0.1 && torque_ki_[i] - ki[i] > -0.1)
            check_counter++;
          if (torque_kd_[i] - kd[i] < 0.1 && torque_kd_[i] - kd[i] > -0.1)
            check_counter++;
          if (torque_i_clamp_[i] - i_clamp[i] < 0.1 && torque_i_clamp_[i] - i_clamp[i] > -0.1)
            check_counter++;
        }
      }
      if (check_counter == (4 * joint_names_.size()))
        ROS_INFO("Torque controller gains check successful");
      else
        ROS_WARN("Torque controller gains check failed");
    }

    ROS_INFO("Hardware initialization status: %s", status_text_);
    h3_handle.setRecording(recording_convention_, recording_mode_);
    h3_handle.Start();
    return true;
  }

  void H3Hardware::read(ros::Time time, ros::Duration period)
  {
    const H3StateHW *hs;
    hs = h3_handle.readH3();
    if (hs->connection_status)
    {
      for (int i = 0; i < joint_names_.size(); i++)
      {
        joint_angles_[i] = hs->joint_angle[i] * M_PI_R_ / M_PI_D_;
        joint_torques_[i] = hs->joint_torque[i];
        joint_velocities_[i] = hs->joint_velocity[i] * M_PI_R_ / M_PI_D_;
        control_type_status_[i] = hs->control_type[i];
        joint_motor_torques_[i] = hs->motor_torque[i];

        if (!(i == 1 || i == 4)) //change the direction of rotation in all joints except knees
        {
          joint_angles_[i] = -joint_angles_[i];
          joint_velocities_[i] = -joint_velocities_[i];
          joint_motor_torques_[i] = -joint_motor_torques_[i];
        }
        else
        {
          joint_torques_[i] = -joint_torques_[i]; //change the direction of torque sensor in knees
        }
      }
      right_heel_sensor_ = hs->right_heel;
      right_toe_sensor_ = hs->right_toe;
      left_heel_sensor_ = hs->left_heel;
      left_toe_sensor_ = hs->left_toe;
      battery_voltage_ = hs->battery_voltage;
      h3_state_TC_ = hs->exo_state;
      trigger_input_ = hs->trigger_input;
      h3_commandApp_ = hs->commandApp;
      trigger_counter_ = hs->trigger_counter;
      h3_runtime_min_ = hs->exo_run_time_min;
      h3_runtime_sec_ = hs->exo_run_time_sec;
      h3_runtime_csec_ = hs->exo_run_time_csec;
      h3_connection_status_ = hs->connection_status;
      record_status_ = hs->record_status;
      ROS_INFO_ONCE("Exo-H3 connected");
    }
    else
    {
      h3_connection_status_ = hs->connection_status;
      ROS_WARN_ONCE("Exo-H3 no connected, try to establish communication");
    }
  }
  void H3Hardware::write(ros::Time time, ros::Duration period)
  {
    /*Enforce limits*/
    position_joint_limit_interface_.enforceLimits(period);
    effort_joint_limit_interface_.enforceLimits(period);

    for (int i = 0; i < joint_names_.size(); ++i)
    {
      /* Control type */
      h3_command_HW_.type_of_control[i] = control_type_[i];

      /* Controller Setpoint*/
      if (!(i == 1 || i == 4)) //change the direction of signals in all joints except knees
      {
        h3_command_HW_.position_setpoint[i] = (signed char)(-position_setpoint_[i] * M_PI_D_ / M_PI_R_);
        h3_command_HW_.torque_setpoint[i] = (signed char)(effort_command_[i]);
        h3_command_HW_.duty_cycle[i] = -effort_command_[i];
        h3_command_HW_.min_angles[i] = (signed char)(-max_angles_[i] * M_PI_D_ / M_PI_R_);
        h3_command_HW_.max_angles[i] = (signed char)(-min_angles_[i] * M_PI_D_ / M_PI_R_);
      }
      else
      {
        h3_command_HW_.position_setpoint[i] = (signed char)(position_setpoint_[i] * M_PI_D_ / M_PI_R_);
        h3_command_HW_.torque_setpoint[i] = (signed char)(-effort_command_[i]);
        h3_command_HW_.duty_cycle[i] = effort_command_[i];
        h3_command_HW_.min_angles[i] = (signed char)(min_angles_[i] * M_PI_D_ / M_PI_R_);
        h3_command_HW_.max_angles[i] = (signed char)(max_angles_[i] * M_PI_D_ / M_PI_R_);
      }
      h3_command_HW_.stiffness_setpoint[i] = stiffness_setpoint_[i];
      h3_command_HW_.percentage_assistance[i] = percentage_assistance_[i];
    }

    if (robot_task_ == h3_definition::TaskControllerCommand::EnableBluetoothAppCommand)
    {
      h3_command_HW_.robot_command[0] = 1;
      h3_command_HW_.robot_command[1] = h3_definition::TaskControllerCommand::EnableBluetoothAppCommand;
    }
    else
    {
      h3_command_HW_.robot_command[0] = 0;
      h3_command_HW_.robot_command[1] = robot_task_;
    }

    h3_command_HW_.trigger_out = trigger_output_;

    h3_command_HW_.base_file_name = base_file_name_;
    h3_command_HW_.start_stop_recording = start_stop_recording_;
    h3_command_HW_.recording_time_sec = recording_time_sec_;
    h3_command_HW_.timed_recording = timed_recording_;

    h3_handle.writeH3(h3_command_HW_);
  }

} // namespace h3_hardware_interface