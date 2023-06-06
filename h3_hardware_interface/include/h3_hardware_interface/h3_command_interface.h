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

#include "h3_hardware_interface/h3_state_interface.h"

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace h3_hardware_interface
{
    struct H3Command
    {

        unsigned char *control_type = nullptr;
        double *position_setpoint = nullptr;
        double *effort_command = nullptr;
        double *stiffness_setpoint = nullptr;
        double *min_angles = nullptr;
        double *max_angles = nullptr;

        unsigned char *percentage_assistance = nullptr;
        unsigned char *robot_tasK = nullptr;
        unsigned char *trigger_output = nullptr;

        std::string *file_name = nullptr;
        int *start_stop_recording = nullptr;
        double *recording_time_sec = nullptr;
        bool *timed_recording = nullptr;
    };

    class H3CommandHandle : public H3StateHandle
    {
    public:
        H3CommandHandle() {}
        H3CommandHandle(const H3StateHandle &h3_state, H3Command &h3_command) : H3StateHandle(h3_state)
        {
            control_type_ = h3_command.control_type;
            position_setpoint_ = h3_command.position_setpoint;
            effort_command_ = h3_command.effort_command;
            stiffness_setpoint_ = h3_command.stiffness_setpoint;
            min_angles_ = h3_command.min_angles;
            max_angles_ = h3_command.max_angles;
            percentage_assistance_ = h3_command.percentage_assistance;
            robot_task_ = h3_command.robot_tasK;
            trigger_output_ = h3_command.trigger_output;

            file_name_ = h3_command.file_name;
            start_stop_recording_ = h3_command.start_stop_recording;
            recording_time_sec_ = h3_command.recording_time_sec;
            timed_recording_ = h3_command.timed_recording;
        }

        ~H3CommandHandle() {}
        void setControlType(unsigned char *control_type)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
                control_type_[i] = control_type[i];
        }

        void setJointAngles(double *position_setpoint)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
                position_setpoint_[i] = position_setpoint[i];
        }

        void setJointAssistances(unsigned char *assistance)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
                percentage_assistance_[i] = assistance[i];
        }

        void setMaxAngles(double *max_angles)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
                max_angles_[i] = max_angles[i];
        }

        void setMinAngles(double *min_angles)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
                min_angles_[i] = min_angles[i];
        }

        void setJointDutyCycles(double *duty_cycle)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
            {
                effort_command_[i] = duty_cycle[i];
            }
        }

        void setJointStiffness(double *stiffness_setpoint)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
                stiffness_setpoint_[i] = stiffness_setpoint[i];
        }

        void setJointTorques(double *torque_setpoint)
        {
            for (int i = 0; i < this->getNumberOfJoints(); i++)
                effort_command_[i] = torque_setpoint[i];
        }

        void setRobotTask(unsigned char robot_task) { *robot_task_ = robot_task; }

        void setTriggerOutput(int mode) { *trigger_output_ = mode; }

        void setRecordState(std::string &base_file_name, int start_stop_recording, double recording_time_sec, bool timed_recording)
        {
            *file_name_ = base_file_name;
            *start_stop_recording_ = start_stop_recording;
            *recording_time_sec_ = recording_time_sec;
            *timed_recording_ = timed_recording;
        }

        unsigned char *getControltypeCommandPtr() { return control_type_; }
        double *getJointAnglesCommandPtr() { return position_setpoint_; }
        double *getJointTorquesCommandPtr() { return effort_command_; }
        double *getJointStiffnessCommand() { return stiffness_setpoint_; }
        double *getMinAnglesCommand() { return min_angles_; }
        double *getMaxAnglesCommand() { return max_angles_; }
        unsigned char *getAssistanceCommand() { return percentage_assistance_; }
        unsigned char getRobotTaskCommand() { return *robot_task_; }

    private:
        unsigned char *control_type_ = nullptr;
        double *position_setpoint_ = nullptr;
        double *effort_command_ = nullptr;
        double *stiffness_setpoint_ = nullptr;
        unsigned char *robot_task_ = nullptr;
        double *min_angles_ = nullptr;
        double *max_angles_ = nullptr;
        unsigned char *percentage_assistance_ = nullptr;
        unsigned char *trigger_output_ = nullptr;

        std::string *file_name_ = nullptr;
        int *start_stop_recording_ = nullptr;
        double *recording_time_sec_ = nullptr;
        bool *timed_recording_ = nullptr;
    };

    // h3_command_interface
    class H3CommandInterface : public hardware_interface::HardwareResourceManager<H3CommandHandle>
    {
    };
} // namespace h3_hardware_interface