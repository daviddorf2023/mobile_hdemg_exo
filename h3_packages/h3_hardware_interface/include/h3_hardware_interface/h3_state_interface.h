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

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace h3_hardware_interface
{
    struct H3State
    {
        H3State() {}
        unsigned int number_of_joints = 0;
        double *joint_angles = nullptr;
        double *joint_velocities = nullptr;
        double *joint_torques = nullptr;
        double *joint_motor_torques = nullptr;
        unsigned char *right_heel_sensor = nullptr;
        unsigned char *right_toe_sensor = nullptr;
        unsigned char *left_heel_sensor = nullptr;
        unsigned char *left_toe_sensor = nullptr;
        double *battery_voltage = nullptr;
        unsigned char *h3_state = nullptr;
        unsigned char *trigger_input = nullptr;
        unsigned char *h3_commandApp = nullptr;
        unsigned char *trigger_counter = nullptr;
        unsigned char *h3_runtime_min = nullptr;
        unsigned char *h3_runtime_sec = nullptr;
        unsigned char *h3_runtime_csec = nullptr;
        unsigned char *h3_connection_status = nullptr;
        unsigned char *control_type = nullptr;

        unsigned char *recording_status = nullptr;
    };

    class H3StateHandle
    {
    public:
        H3StateHandle() = default;
        H3StateHandle(const H3State &H3State, const std::string &robot_name, std::vector<std::string> joint_name)
        {
            joint_names_ = joint_name;
            robot_name_ = robot_name;
            number_of_joints_ = H3State.number_of_joints;
            joint_angles_ = H3State.joint_angles;
            joint_velocities_ = H3State.joint_velocities;
            joint_torques_ = H3State.joint_torques;
            joint_motor_torques_ = H3State.joint_motor_torques;
            right_heel_sensor_ = H3State.right_heel_sensor;
            right_toe_sensor_ = H3State.right_toe_sensor;
            left_heel_sensor_ = H3State.left_heel_sensor;
            left_toe_sensor_ = H3State.left_toe_sensor;
            battery_voltage_ = H3State.battery_voltage;
            h3_state_ = H3State.h3_state;
            trigger_input_ = H3State.trigger_input;
            h3_commandApp_ = H3State.h3_commandApp;
            trigger_counter_ = H3State.trigger_counter;
            h3_runtime_min_ = H3State.h3_runtime_min;
            h3_runtime_sec_ = H3State.h3_runtime_sec;
            h3_runtime_csec_ = H3State.h3_runtime_csec;
            h3_connection_status_ = H3State.h3_connection_status;
            control_type_ = H3State.control_type;
            recording_status_ = H3State.recording_status;
        }

        ~H3StateHandle(){};

        std::string getName() const { return robot_name_; }
        std::vector<std::string> getJointNames() const {return joint_names_; }
        int getNumberOfJoints() const { return number_of_joints_; }

        /**
         * @brief Get a pointer to an storage of joint angles
         * 
         * @return * double* 
         */
        double *getJointAngles() { return joint_angles_; }
        double *getJointTorques() { return joint_torques_; }
        double *getJointVelocities() { return joint_velocities_; }
        double *getJointMotorTorques() { return joint_motor_torques_; }
        unsigned char getRightHeelSensor() { return *right_heel_sensor_; }
        unsigned char getRightToeSensor() { return *right_toe_sensor_; }
        unsigned char getLeftHeelSensor() { return *left_heel_sensor_; }
        unsigned char getLeftToeSensor() { return *left_toe_sensor_; }
        double getBatteryVoltage() { return *battery_voltage_; }
        unsigned char getH3State() { return *h3_state_; }
        unsigned char getTriggerInput() { return *trigger_input_; }
        unsigned char getH3CommandApp() { return *h3_commandApp_; }
        unsigned char getTriggerCounter() { return *trigger_counter_; }
        unsigned char getH3RuntimeMin() { return *h3_runtime_min_; }
        unsigned char getH3RuntimeSec() { return *h3_runtime_sec_; }
        unsigned char getH3RuntimeCSec() { return *h3_runtime_csec_; }
        unsigned char getExoConnectionStatus() { return *h3_connection_status_; }
        unsigned char* getExoControlType() { return control_type_; }
        unsigned char getRecordingStatus() { return *recording_status_; }

    private:
        std::string robot_name_;
        std::vector<std::string> joint_names_;
        unsigned int number_of_joints_;
        double *joint_angles_;
        double *joint_velocities_;
        double *joint_torques_;
        double *joint_motor_torques_;
        unsigned char *right_heel_sensor_;
        unsigned char *right_toe_sensor_;
        unsigned char *left_heel_sensor_;
        unsigned char *left_toe_sensor_;
        double *battery_voltage_;
        unsigned char *h3_state_;
        unsigned char *trigger_input_;
        unsigned char *h3_commandApp_;
        unsigned char *trigger_counter_;
        unsigned char *h3_runtime_min_;
        unsigned char *h3_runtime_sec_;
        unsigned char *h3_runtime_csec_;
        unsigned char *h3_connection_status_;
        unsigned char *control_type_;
        unsigned char *recording_status_;
    };

    class H3StateInterface : public hardware_interface::HardwareResourceManager<H3StateHandle>
    {
    };

} // namespace h3_hardware_interface