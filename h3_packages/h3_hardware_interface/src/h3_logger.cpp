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

#include "h3_hardware_interface/h3_logger.h"

namespace h3_hardware_interface
{
    void H3Logger::recordStateHW(const H3StateHW *state_hw, const H3CommandHW *h3_command, int recording_convention, int recording_mode, std::string &base_file_name, unsigned int trigger_mode, double record_time_sec, bool timed_recording)
    {
        record_time_sec_ = record_time_sec;
        recording_convention_ = recording_convention;
        recording_mode_ = recording_mode;
        auto time_now_sec = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000.0;

        if (!file_.is_open())
        {
            base_file_name_ = base_file_name;
        }
        if (trigger_mode == RecordTriggerMode::SWStop)
        {
            record_data_ = false;
        }
        if (trigger_mode == RecordTriggerMode::SwStartStop && !timed_recording)
        {
            record_data_ = true;
            if (!file_.is_open())
            {
                record_counter_++;
                fileOpen(base_file_name_);
            }
        }
        /* Data recording triggered by software*/
        if ((record_mode_k_1 != trigger_mode) && trigger_mode == RecordTriggerMode::SwStartStop && timed_recording)
        {
            record_data_ = true;
            if (!file_.is_open())
            {
                record_counter_++;
                fileOpen(base_file_name_);
            }
        }
        record_mode_k_1 = trigger_mode;

        /* Data recording triggered by a pulse on input pin */
        if (trigger_mode == RecordTriggerMode::TriggerInputPulse)
        {
            // Detect rising edge
            if (trigger_input_1_ == false && state_hw->trigger_input == 1 && trigger_mode == RecordTriggerMode::TriggerInputPulse && (time_now_sec - trigger_input_time_ > 1.0))
            {
                record_status_ = RecordStatus::WaitingInputTrigger;
                trigger_input_time_ = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000.0;
                record_data_ ^= true;
                if (!file_.is_open())
                {
                    record_counter_++;
                    fileOpen(base_file_name_);
                }
            }
            trigger_input_1_ = (bool)state_hw->trigger_input;
        }
        /* Data recording triggered by a high on input pin */
        if (trigger_mode == RecordTriggerMode::TriggerInputHigh)
        {
            record_status_ = RecordStatus::WaitingInputTrigger;
            if (state_hw->trigger_input == 1)
            {
                record_data_ = true;
                if (!file_.is_open())
                {
                    record_counter_++;
                    fileOpen(base_file_name_);
                }
            }
            else
            {
                record_data_ = false;
            }
        }
        /* Data recording triggered by a pulse on output pin */
        if (trigger_mode == RecordTriggerMode::TriggerOutputPulse)
        {
            //detect rising edge
            if (trigger_output_1_ == false && h3_command->trigger_out == 1 && trigger_mode == RecordTriggerMode::TriggerOutputPulse)
            {
                record_status_ = RecordStatus::WaitingOutputTrigger;
                record_data_ ^= true;
                if (!file_.is_open())
                {
                    record_counter_++;
                    fileOpen(base_file_name_);
                }
#
                //std::cout << "trigger" << std::endl;
            }
            trigger_output_1_ = (bool)h3_command->trigger_out;
        }

        /* Data recording triggered by a high on output pin */
        if (trigger_mode == RecordTriggerMode::TriggerOutputHigh)
        {
            record_status_ = RecordStatus::WaitingOutputTrigger;
            if (h3_command->trigger_out == 2)
            {
                record_data_ = true;
                if (!file_.is_open())
                {
                    record_counter_++;
                    fileOpen(base_file_name_);
                }
            }
            else
            {
                record_data_ = false;
            }
        }

        // record_data
        if (record_data_)
        {
            if (file_.is_open())
            {
                fileDataSave(state_hw, h3_command);
                record_status_ = RecordStatus::Recording;
                if (timed_recording == true && (time_now_sec - recording_start_sec_) >= record_time_sec_)
                {
                    record_data_ = false;
                }
            }
        }
        else
        {
            if (file_.is_open())
            {
                file_.close();
            }
            if (trigger_mode == RecordTriggerMode::SwStartStop || trigger_mode == RecordTriggerMode::SWStop)
            {
                record_status_ = RecordStatus::NoRecording;
            }
            else if (trigger_mode == RecordTriggerMode::TriggerInputPulse || trigger_mode == RecordTriggerMode::TriggerInputHigh)
            {
                record_status_ = RecordStatus::WaitingInputTrigger;
            }
            else if (trigger_mode == RecordTriggerMode::TriggerOutputPulse || trigger_mode == RecordTriggerMode::TriggerOutputHigh)
            {
                record_status_ = RecordStatus::WaitingOutputTrigger;
            }
        }
    }

    /* Open */
    void H3Logger::fileOpen(std::string &file_name)
    {
        auto t_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        file_.open(file_name + "_" + std::to_string(record_counter_) + ".csv", std::ios::out | std::ios::app);
        std::tm now_tm = *std::localtime(&t_c);
        char date_string[100];
        strftime(date_string, 50, "%y_%m_%d_%H_%M_%S", &now_tm);
        //file << str_p_time;
        if (recording_convention_ == RecordingConvention::H3)
        {
            if (recording_mode_ == RecordingMode::Inputs || recording_mode_ == RecordingMode::InputsOutputs)
            {
                file_ << "Technaid S.L-Exo-H3"
                      << "\n";
                file_ << "Date"
                      << ","
                      << date_string
                      << "\n";
                file_ << "PC_time_[s]"
                      << ","
                      << "Exo_run_time_[ms]"
                      << ","
                      << "Capture_counter"
                      << ","
                      << "Command_app"
                      << ","
                      << "Exo_state"
                      << ","
                      << "Right_hip_angle_[°]"
                      << ","
                      << "Right_knee_angle_[°]"
                      << ","
                      << "Right_ankle_angle_[°]"
                      << ","
                      << "Left_hip_angle_[°]"
                      << ","
                      << "Left_knee_angle_[°]"
                      << ","
                      << "Left_ankle_angle_[°]"
                      << ","
                      << "Right_hip_velocity_[°/s]"
                      << ","
                      << "Right_knee_velocity_[°/s]"
                      << ","
                      << "Right_ankle_velocity_[°/s]"
                      << ","
                      << "Left_hip_velocity_[°/s]"
                      << ","
                      << "Left_knee_velocity_[°/s]"
                      << ","
                      << "Left_ankle_velocity_[°/s]"
                      << ","
                      << "Right_hip_torque_sensor[Nm]"
                      << ","
                      << "Right_knee_torque_sensor[Nm]"
                      << ","
                      << "Right_ankle_torque_sensor[Nm]"
                      << ","
                      << "Left_hip_torque_sensor[Nm]"
                      << ","
                      << "Left_knee_torque_sensor[Nm]"
                      << ","
                      << "Left_ankle_torque_sensor[Nm]"
                      << ","
                      << "Right_hip_motor_torque_[Nm]"
                      << ","
                      << "Right_knee_motor_torque_[Nm]"
                      << ","
                      << "Right_ankle_motor_torque_[Nm]"
                      << ","
                      << "Left_hip_motor_torque_[Nm]"
                      << ","
                      << "Left_knee_motor_torque_[Nm]"
                      << ","
                      << "Left_ankle_motor_torque_[Nm]"
                      << ","
                      << "Right_heel_pressure_value"
                      << ","
                      << "Right_toe_pressure_value"
                      << ","
                      << "Left_heel_pressure_value"
                      << ","
                      << "Left_toe_pressure_value"
                      << ","
                      << "Battery_voltage_[VDC]"
                      << ","
                      << "Trigger input";
            }
            if (recording_mode_ == RecordingMode::InputsOutputs)
            {
                file_ << ","
                      << "Right_hip_control_type"
                      << ","
                      << "Right_knee_control_type"
                      << ","
                      << "Right_ankle_control_type"
                      << ","
                      << "Left_hip_control_type"
                      << ","
                      << "Left_knee_control_type"
                      << ","
                      << "Left_ankle_control_type"
                      << ","
                      << "Right_hip_assistance_[%]"
                      << ","
                      << "Right_knee_assistance_[%]"
                      << ","
                      << "Right_ankle_assistance_[%]"
                      << ","
                      << "Left_hip_assistance_[%]"
                      << ","
                      << "Left_knee_assistance_[%]"
                      << ","
                      << "Left_ankle_assistance_[%]"
                      << ","
                      << "Right_hip_position_setpoint_[°]"
                      << ","
                      << "Right_knee_position_setpoint_[°]"
                      << ","
                      << "Right_ankle_position_setpoint_[°]"
                      << ","
                      << "Left_hip_position_setpoint_[°]"
                      << ","
                      << "Left_knee_position_setpoint_[°]"
                      << ","
                      << "Left_ankle_position_setpoint_[°]"
                      << ","
                      << "Right_hip_stiffness_setpoint_[%]"
                      << ","
                      << "Right_knee_stiffness_setpoint_[%]"
                      << ","
                      << "Right_ankle_stiffness_setpoint_[%]"
                      << ","
                      << "Left_hip_stiffness_setpoint_[%]"
                      << ","
                      << "Left_knee_stiffness_setpoint_[%]"
                      << ","
                      << "Left_ankle_stiffness_setpoint_[%]"
                      << ","
                      << "Right_hip_torque_setpoint_[Nm]"
                      << ","
                      << "Right_knee_torque_setpoint_[Nm]"
                      << ","
                      << "Right_ankle_torque_setpoint_[Nm]"
                      << ","
                      << "Left_hip_torque_setpoint_[Nm]"
                      << ","
                      << "Left_knee_torque_setpoint_[Nm]"
                      << ","
                      << "Left_ankle_torque_setpoint_[Nm]"
                      << ","
                      << "Right_hip_duty_cycle_[%]"
                      << ","
                      << "Right_knee_duty_cycle_[%]"
                      << ","
                      << "Right_ankle_duty_cycle_[%]"
                      << ","
                      << "Left_hip_duty_cycle_[%]"
                      << ","
                      << "Left_knee_duty_cycle_[%]"
                      << ","
                      << "Left_ankle_duty_cycle_[%]";
            }
            file_ << "\n";
        }
        if (recording_convention_ == RecordingConvention::ROS)
        {
            if (recording_mode_ == RecordingMode::Inputs || recording_mode_ == RecordingMode::InputsOutputs)
            {
                file_ << "Technaid S.L-Exo-H3"
                      << "\n";
                file_ << "Date"
                      << ","
                      << date_string
                      << "\n";
                file_ << "PC_time_[s]"
                      << ","
                      << "Exo_run_time_[ms]"
                      << ","
                      << "Capture_counter"
                      << ","
                      << "Command_app"
                      << ","
                      << "Exo_state"
                      << ","
                      << "Right_hip_angle_[rad]"
                      << ","
                      << "Right_knee_angle_[rad]"
                      << ","
                      << "Right_ankle_angle_[rad]"
                      << ","
                      << "Left_hip_angle_[rad]"
                      << ","
                      << "Left_knee_angle_[rad]"
                      << ","
                      << "Left_ankle_angle_[rad]"
                      << ","
                      << "Right_hip_velocity_[rad/s]"
                      << ","
                      << "Right_knee_velocity_[rad/s]"
                      << ","
                      << "Right_ankle_velocity_[rad/s]"
                      << ","
                      << "Left_hip_velocity_[rad/s]"
                      << ","
                      << "Left_knee_velocity_[rad/s]"
                      << ","
                      << "Left_ankle_velocity_[rad/s]"
                      << ","
                      << "Right_hip_torque_sensor_[Nm]"
                      << ","
                      << "Right_knee_torque_sensor_[Nm]"
                      << ","
                      << "Right_ankle_torque_sensor_[Nm]"
                      << ","
                      << "Left_hip_torque_sensor_[Nm]"
                      << ","
                      << "Left_knee_torque_sensor[Nm]"
                      << ","
                      << "Left_ankle_torque_sensor_[Nm]"
                      << ","
                      << "Right_hip_motor_torque_[Nm]"
                      << ","
                      << "Right_knee_motor_torque_[Nm]"
                      << ","
                      << "Right_ankle_motor_torque_[Nm]"
                      << ","
                      << "Left_hip_motor_torque_[Nm]"
                      << ","
                      << "Left_knee_motor_torque_[Nm]"
                      << ","
                      << "Left_ankle_motor_torque_[Nm]"
                      << ","
                      << "Right_heel_pressure_value"
                      << ","
                      << "Right_toe_pressure_value"
                      << ","
                      << "Left_heel_pressure_value"
                      << ","
                      << "Left_toe_pressure_value"
                      << ","
                      << "Battery_voltage_[VDC]"
                      << ","
                      << "Trigger input";
            }
            if (recording_mode_ == RecordingMode::InputsOutputs)
            {
                file_ << ","
                      << "Right_hip_control_type"
                      << ","
                      << "Right_knee_control_type"
                      << ","
                      << "Right_ankle_control_type"
                      << ","
                      << "Left_hip_control_type"
                      << ","
                      << "Left_knee_control_type"
                      << ","
                      << "Left_ankle_control_type"
                      << ","
                      << "Right_hip_assistance_[%]"
                      << ","
                      << "Right_knee_assistance_[%]"
                      << ","
                      << "Right_ankle_assistance_[%]"
                      << ","
                      << "Left_hip_assistance_[%]"
                      << ","
                      << "Left_knee_assistance_[%]"
                      << ","
                      << "Left_ankle_assistance_[%]"
                      << ","
                      << "Right_hip_position_setpoint_[rad]"
                      << ","
                      << "Right_knee_position_setpoint_[rad]"
                      << ","
                      << "Right_ankle_position_setpoint_[rad]"
                      << ","
                      << "Left_hip_position_setpoint_[rad]"
                      << ","
                      << "Left_knee_position_setpoint_[rad]"
                      << ","
                      << "Left_ankle_position_setpoint_[rad]"
                      << ","
                      << "Right_hip_stiffness_setpoint_[%]"
                      << ","
                      << "Right_knee_stiffness_setpoint_[%]"
                      << ","
                      << "Right_ankle_stiffness_setpoint_[%]"
                      << ","
                      << "Left_hip_stiffness_setpoint_[%]"
                      << ","
                      << "Left_knee_stiffness_setpoint_[%]"
                      << ","
                      << "Left_ankle_stiffness_setpoint_[%]"
                      << ","
                      << "Right_hip_torque_setpoint_[Nm]"
                      << ","
                      << "Right_knee_torque_setpoint_[Nm]"
                      << ","
                      << "Right_ankle_torque_setpoint_[Nm]"
                      << ","
                      << "Left_hip_torque_setpoint_[Nm]"
                      << ","
                      << "Left_knee_torque_setpoint_[Nm]"
                      << ","
                      << "Left_ankle_torque_setpoint_[Nm]"
                      << ","
                      << "Right_hip_duty_cycle_[%]"
                      << ","
                      << "Right_knee_duty_cycle_[%]"
                      << ","
                      << "Right_ankle_duty_cycle_[%]"
                      << ","
                      << "Left_hip_duty_cycle_[%]"
                      << ","
                      << "Left_knee_duty_cycle_[%]"
                      << ","
                      << "Left_ankle_duty_cycle_[%]";
            }
            file_ << "\n";
        }
        recording_start_sec_ = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000.0;
    }

    void H3Logger::fileDataSave(const H3StateHW *h3_data, const H3CommandHW *h3_command) //std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())-> 1 seg
    {
        if (recording_convention_ == RecordingConvention::H3)
        {
            auto pc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            file_ << std::fixed;
            file_ << std::setprecision(3);
            file_ << pc_ms.count() / 1000.0 << ","
                  << h3_data->exo_run_time_min * 60000 + h3_data->exo_run_time_sec * 1000 + h3_data->exo_run_time_csec * 10 << ","
                  << (int)h3_data->trigger_counter << ","
                  << (int)h3_data->commandApp << ","
                  << (int)h3_data->exo_state << ","
                  << h3_data->joint_angle[0] << ","
                  << h3_data->joint_angle[1] << ","
                  << h3_data->joint_angle[2] << ","
                  << h3_data->joint_angle[3] << ","
                  << h3_data->joint_angle[4] << ","
                  << h3_data->joint_angle[5] << ","
                  << h3_data->joint_velocity[0] << ","
                  << h3_data->joint_velocity[1] << ","
                  << h3_data->joint_velocity[2] << ","
                  << h3_data->joint_velocity[3] << ","
                  << h3_data->joint_velocity[4] << ","
                  << h3_data->joint_velocity[5] << ","
                  << h3_data->joint_torque[0] << ","
                  << h3_data->joint_torque[1] << ","
                  << h3_data->joint_torque[2] << ","
                  << h3_data->joint_torque[3] << ","
                  << h3_data->joint_torque[4] << ","
                  << h3_data->joint_torque[5] << ","
                  << h3_data->motor_torque[0] << ","
                  << h3_data->motor_torque[1] << ","
                  << h3_data->motor_torque[2] << ","
                  << h3_data->motor_torque[3] << ","
                  << h3_data->motor_torque[4] << ","
                  << h3_data->motor_torque[5] << ","
                  << (int)h3_data->right_heel << ","
                  << (int)h3_data->right_toe << ","
                  << (int)h3_data->left_heel << ","
                  << (int)h3_data->left_toe << ","
                  << h3_data->battery_voltage << ","
                  << (int)h3_data->trigger_input << ",";
            if (recording_mode_ == RecordingMode::InputsOutputs)
            {

                file_ << (int)h3_command->type_of_control[0] << ","
                      << (int)h3_command->type_of_control[1] << ","
                      << (int)h3_command->type_of_control[2] << ","
                      << (int)h3_command->type_of_control[3] << ","
                      << (int)h3_command->type_of_control[4] << ","
                      << (int)h3_command->type_of_control[5] << ","
                      << (int)h3_command->percentage_assistance[0] << ","
                      << (int)h3_command->percentage_assistance[1] << ","
                      << (int)h3_command->percentage_assistance[2] << ","
                      << (int)h3_command->percentage_assistance[3] << ","
                      << (int)h3_command->percentage_assistance[4] << ","
                      << (int)h3_command->percentage_assistance[5] << ","
                      << (int)h3_command->position_setpoint[0] << ","
                      << (int)h3_command->position_setpoint[1] << ","
                      << (int)h3_command->position_setpoint[2] << ","
                      << (int)h3_command->position_setpoint[3] << ","
                      << (int)h3_command->position_setpoint[4] << ","
                      << (int)h3_command->position_setpoint[5] << ","
                      << (int)h3_command->stiffness_setpoint[0] << ","
                      << (int)h3_command->stiffness_setpoint[1] << ","
                      << (int)h3_command->stiffness_setpoint[2] << ","
                      << (int)h3_command->stiffness_setpoint[3] << ","
                      << (int)h3_command->stiffness_setpoint[4] << ","
                      << (int)h3_command->stiffness_setpoint[5] << ","
                      << (int)h3_command->torque_setpoint[0] << ","
                      << (int)h3_command->torque_setpoint[1] << ","
                      << (int)h3_command->torque_setpoint[2] << ","
                      << (int)h3_command->torque_setpoint[3] << ","
                      << (int)h3_command->torque_setpoint[4] << ","
                      << (int)h3_command->torque_setpoint[5] << ","
                      << (int)h3_command->duty_cycle[0] << ","
                      << (int)h3_command->duty_cycle[1] << ","
                      << (int)h3_command->duty_cycle[2] << ","
                      << (int)h3_command->duty_cycle[3] << ","
                      << (int)h3_command->duty_cycle[4] << ","
                      << (int)h3_command->duty_cycle[5];
            }
            file_ << "\n";
        }

        if (recording_convention_ == RecordingConvention::ROS)
        {
            double k = M_PI_R_ / M_PI_D_;
            auto pc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            file_ << std::fixed;
            file_ << std::setprecision(3);
            file_ << pc_ms.count() / 1000.0 << ","
                  << h3_data->exo_run_time_min * 60000 + h3_data->exo_run_time_sec * 1000 + h3_data->exo_run_time_csec * 10 << ","
                  << (int)h3_data->trigger_counter << ","
                  << (int)h3_data->commandApp << ","
                  << (int)h3_data->exo_state << ","
                  << h3_data->joint_angle[0] * (-k) << ","
                  << h3_data->joint_angle[1] * (k) << ","
                  << h3_data->joint_angle[2] * (-k) << ","
                  << h3_data->joint_angle[3] * (-k) << ","
                  << h3_data->joint_angle[4] * (k) << ","
                  << h3_data->joint_angle[5] * (-k) << ","
                  << h3_data->joint_velocity[0] * (-k) << ","
                  << h3_data->joint_velocity[1] * (k) << ","
                  << h3_data->joint_velocity[2] * (-k) << ","
                  << h3_data->joint_velocity[3] * (-k) << ","
                  << h3_data->joint_velocity[4] * (k) << ","
                  << h3_data->joint_velocity[5] * (-k) << ","
                  << h3_data->joint_torque[0] << ","
                  << h3_data->joint_torque[1] * (-1.000) << ","
                  << h3_data->joint_torque[2] << ","
                  << h3_data->joint_torque[3] << ","
                  << h3_data->joint_torque[4] * (-1.000) << ","
                  << h3_data->joint_torque[5] << ","
                  << h3_data->motor_torque[0] * (-k) << ","
                  << h3_data->motor_torque[1] * (k) << ","
                  << h3_data->motor_torque[2] * (-k) << ","
                  << h3_data->motor_torque[3] * (-k) << ","
                  << h3_data->motor_torque[4] * (k) << ","
                  << h3_data->motor_torque[5] * (-k) << ","
                  << (int)h3_data->right_heel << ","
                  << (int)h3_data->right_toe << ","
                  << (int)h3_data->left_heel << ","
                  << (int)h3_data->left_toe << ","
                  << h3_data->battery_voltage << ","
                  << (int)h3_data->trigger_input << ",";
            if (recording_mode_ == RecordingMode::InputsOutputs)
            {

                file_ << (int)h3_command->type_of_control[0] << ","
                      << (int)h3_command->type_of_control[1] << ","
                      << (int)h3_command->type_of_control[2] << ","
                      << (int)h3_command->type_of_control[3] << ","
                      << (int)h3_command->type_of_control[4] << ","
                      << (int)h3_command->type_of_control[5] << ","
                      << (int)h3_command->percentage_assistance[0] << ","
                      << (int)h3_command->percentage_assistance[1] << ","
                      << (int)h3_command->percentage_assistance[2] << ","
                      << (int)h3_command->percentage_assistance[3] << ","
                      << (int)h3_command->percentage_assistance[4] << ","
                      << (int)h3_command->percentage_assistance[5] << ","
                      << (double)h3_command->position_setpoint[0] * (-k) << ","
                      << (double)h3_command->position_setpoint[1] * (k) << ","
                      << (double)h3_command->position_setpoint[2] * (-k) << ","
                      << (double)h3_command->position_setpoint[3] * (-k) << ","
                      << (double)h3_command->position_setpoint[4] * (k) << ","
                      << (double)h3_command->position_setpoint[5] * (-k) << ","
                      << (int)h3_command->stiffness_setpoint[0] << ","
                      << (int)h3_command->stiffness_setpoint[1] << ","
                      << (int)h3_command->stiffness_setpoint[2] << ","
                      << (int)h3_command->stiffness_setpoint[3] << ","
                      << (int)h3_command->stiffness_setpoint[4] << ","
                      << (int)h3_command->stiffness_setpoint[5] << ","
                      << (int)h3_command->torque_setpoint[0] << ","
                      << (int)h3_command->torque_setpoint[1] * (-1) << ","
                      << (int)h3_command->torque_setpoint[2] << ","
                      << (int)h3_command->torque_setpoint[3] << ","
                      << (int)h3_command->torque_setpoint[4] * (-1) << ","
                      << (int)h3_command->torque_setpoint[5] << ","
                      << (double)h3_command->duty_cycle[0] * (-1) << ","
                      << (double)h3_command->duty_cycle[1] << ","
                      << (double)h3_command->duty_cycle[2] * (-1) << ","
                      << (double)h3_command->duty_cycle[3] * (-1) << ","
                      << (double)h3_command->duty_cycle[4] << ","
                      << (double)h3_command->duty_cycle[5] * (-1);
            }
            file_ << "\n";
        }
    }

    void H3Logger::fileClose()
    {
        file_.close();
    }

    int H3Logger::getRecordingStatus()
    {
        return record_status_;
    }

} // namespace h3_hardware_interface