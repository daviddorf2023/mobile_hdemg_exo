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

#include "h3_hardware_interface/h3_definition.h"
#include "h3_hardware_interface/h3_handle.h"
#include "h3_hardware_interface/h3_state_interface.h"
#include "h3_hardware_interface/stiffness_joint_interface.h"
#include "h3_hardware_interface/h3_command_interface.h"

#include <vector>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <hardware_interface/robot_hw.h>

namespace h3_hardware_interface
{
  /**
 * @brief EXO-H3 hardware interface. This class inherits from the Robot Hardware
 * class from the ros_control package. This class implement the following interfaces with Exo-H3
 * hardware_interface::JointStateInterface
 * hardware_interface::PositionJointInterface
 * hardware_interface::EffortJointInterface
 * hardware_interface::StiffnessJointInterface
 * h3_hardware_interface::H3CommandInterface;
 * h3_hardware_interface::H3StateInterface;
 */

  class H3Hardware : public hardware_interface::RobotHW
  {
  public:
    H3Hardware(ros::NodeHandle &nh);
    ~H3Hardware();

    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    bool initialize();

    /**
     * @brief Read from Hw Handle
     * 
     */
    void read(ros::Time time, ros::Duration period);

    /**
     * @brief Write to HW Handle
     * 
     */
    void write(ros::Time time, ros::Duration period);

    /**
     * @brief Get CAN error status in text format
     * @param status_code Status code 
     * @param error_text null terminated char array
     */
    void getErrorStatus(int status_code, char *error_text);

  private:
    ros::NodeHandle &nh_;

    /**
   * @brief Standard interfaces compliant with Exo-H3.
   *)
   */
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;
    hardware_interface::StiffnessJointInterface stiffness_joint_interface_;
    joint_limits_interface::PositionJointSaturationInterface position_joint_limit_interface_;
    joint_limits_interface::EffortJointSaturationInterface effort_joint_limit_interface_;

    /**
     * @brief Specific Exo-H3 interfaces
     * 
     */
    h3_hardware_interface::H3CommandInterface h3_command_interface_;
    h3_hardware_interface::H3StateInterface h3_state_interface_;

    /**
     * @brief Joint limits
     * 
     */
    std::vector<joint_limits_interface::JointLimits> joint_limits_;

    /**
     * @brief Conversion constant
     * 
     */
    const double M_PI_R_ = 3.14159265358979323846;
    const double M_PI_D_ = 180.00000000000000000000;

    H3State h3_state_;
    std::string robot_name_ = "exo_h3";
    unsigned int number_of_joints_ = 6;
    double joint_angles_[6]; // joint angles
    double joint_velocities_[6];
    double joint_torques_[6]; // gauges torque.
    double joint_motor_torques_[6];
    unsigned char right_heel_sensor_;
    unsigned char right_toe_sensor_;
    unsigned char left_heel_sensor_;
    unsigned char left_toe_sensor_;
    double battery_voltage_;
    unsigned char h3_state_TC_;
    unsigned char trigger_input_;
    unsigned char h3_commandApp_;
    unsigned char trigger_counter_;
    unsigned char h3_runtime_min_;
    unsigned char h3_runtime_sec_;
    unsigned char h3_runtime_csec_;
    unsigned char h3_connection_status_;
    unsigned char control_type_status_[6];
    unsigned char record_status_ = 0;

    H3Command h3_command_;
    H3CommandHW h3_command_HW_;
    signed char joint_control_[6] = {0};
    unsigned char control_type_[6] = {0, 0, 0, 0, 0, 0};
    double position_setpoint_[6] = {0};
    double effort_command_[6] = {0, 0, 0, 0, 0, 0}; //torque and dutyCyle
    double stiffness_setpoint_[6];
    unsigned char robot_task_ = 0;
    double min_angles_[6] = {-100 * M_PI_R_ / M_PI_D_, 0, -25 * M_PI_R_ / M_PI_D_, -100 * M_PI_R_ / M_PI_D_, 0, -25 * M_PI_R_ / M_PI_D_};                                       // H: -100 K: 0 A: -25;
    double max_angles_[6] = {25 * M_PI_R_ / M_PI_D_, 100 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_, 100 * M_PI_R_ / M_PI_D_, 25 * M_PI_R_ / M_PI_D_}; //H: 25, K: 100, A: 25;
    unsigned char percentage_assistance_[6] = {100, 100, 100, 100, 100, 100};

    unsigned char communication_mode_[6];
    unsigned char trigger_output_ = 0;
    unsigned char robot_command_[6];

    std::string base_file_name_ = "h3_data_test";
    int start_stop_recording_ = 0;
    double recording_time_sec_ = 0;
    bool timed_recording_ = false;
    int recording_mode_ = 0;
    int recording_convention_ = 0;

    std::vector<std::string> joint_names_ = {"right_hip", "right_knee", "right_ankle", "left_hip", "left_knee", "left_ankle"};
    bool modify_internal_controllers_ = false;
    std::vector<double> position_kp_ = {180.0, 180.0, 180.0, 180.0, 180.0, 180.0};
    std::vector<double> position_ki_ = {1.9, 1.9, 1.9, 1.9, 1.9, 1.9};
    std::vector<double> position_kd_ = {2.9, 2.9, 2.9, 2.9, 2.9, 2.9};
    std::vector<double> position_i_clamp_ = {80.0, 80.0, 80.0, 80.0, 80.0, 80.0};
    std::vector<double> stiffness_kp_ = {180.0, 180.0, 180.0, 180.0, 180.0, 180.0};
    std::vector<double> stiffness_ki_ = {0, 0, 0, 0, 0, 0};
    std::vector<double> stiffness_kd_ = {2.9, 2.9, 2.9, 2.9, 2.9, 2.9};
    std::vector<double> stiffness_i_clamp_ = {80.0, 80.0, 80.0, 80.0, 80.0, 80.0};
    std::vector<double> torque_kp_ = {100.0, 100.0, 50.0, 100.0, 100.0, 50.0};
    std::vector<double> torque_ki_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::vector<double> torque_kd_ = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
    std::vector<double> torque_i_clamp_ = {40.0, 40.0, 40.0, 40.0, 40.0, 40.0};

    H3Handle h3_handle;

    int CAN_port_ = 81;
    int external_CAN_mode_ = 1; // The Main Controller sends the measurement of the sensors with, 1-> 8-bit of resolution, 3-> 16-bit of resolution
    char status_text_[60];

  }; // class

} // namespace h3_hardware_interface