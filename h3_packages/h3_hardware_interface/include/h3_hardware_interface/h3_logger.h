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

#include "h3_hardware_interface/h3_handle.h"

#include <iomanip>
#include <iostream> //record H3 data
#include <fstream>

namespace h3_hardware_interface
{

    class H3Logger
    {

    public:
        H3Logger(/* args */){};
        ~H3Logger(){};

        void recordStateHW(const H3StateHW *state_hw, const H3CommandHW *h3_command, int recording_convention, int recording_mode, std::string &base_file_name, unsigned int trigger_mode, double record_time_sec, bool timed_recording);

        int getRecordingStatus();

    private:
        void fileOpen(std::string &file_name);
        void fileClose();
        void fileDataSave(const H3StateHW *h3_data, const H3CommandHW *h3_command);

        const double M_PI_R_ = 3.14159265358979323846;
        const double M_PI_D_ = 180.00000000000000000000;

        // Declaration for input record data
        std::fstream file_;
        bool record_data_ = false;
        unsigned int record_counter_ = 0;
        bool trigger_input_1_ = false;
        bool trigger_output_1_ = false;
        double recording_start_sec_ = 0;
        std::string base_file_name_;
        double record_time_sec_ = 0;
        unsigned int record_mode_k_1;
        double trigger_input_time_ = 0;

        int record_status_ = 0; // 0-> no recording, 1->recording 2->wait input trigger 3->wait ouput trigger

        int recording_convention_ = 0;
        int recording_mode_ = 0;

        enum RecordTriggerMode //recoding trrigger mode
        {
            SWStop,
            SwStartStop,
            TriggerInputPulse,
            TriggerInputHigh,
            TriggerOutputPulse,
            TriggerOutputHigh,
        };
        enum RecordStatus
        {
            NoRecording,
            Recording,
            WaitingInputTrigger,
            WaitingOutputTrigger,
        };
        enum RecordingMode
        {
            Inputs,
            InputsOutputs,
        };
        enum RecordingConvention
        {
            H3,
            ROS,
        };
    };

} // namespace h3_hardware_interface
