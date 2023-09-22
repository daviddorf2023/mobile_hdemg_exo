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

#define ENGLISH 0x09 // Define PCAN error string language

#include "h3_definition.h"

#include <iostream>

#include <thread>
#include <vector>
#include <chrono>
#include <mutex>

#include <PCANBasic.h>

namespace h3_hardware_interface
{
  /**
   * @brief Storage of status data received from the H3 Main Controller.
  */
  struct H3StateHW
  {
    H3StateHW() {}
    double joint_angle[6] = {0};
    double joint_velocity[6] = {0};
    double joint_torque[6] = {0};
    double motor_torque[6] = {0};
    unsigned char right_heel = {0};
    unsigned char right_toe = {0};
    unsigned char left_heel = {0};
    unsigned char left_toe = {0};
    double battery_voltage = {0};
    unsigned char exo_state = {0};
    unsigned char trigger_input = {0};
    unsigned char commandApp = {0};
    unsigned char trigger_counter = {0};
    unsigned char exo_run_time_min = {0};
    unsigned char exo_run_time_sec = {0};
    unsigned char exo_run_time_csec = {0};

    //Program execution indicators
    unsigned char control_type[6] = {0};
    unsigned char connection_status = 0;
    unsigned char record_status = 0;
  };

  /**
 * @brief Storage of command data sent to the H3 Main Controller.
 *
 */
  struct H3CommandHW
  {
    H3CommandHW() {}
    signed char joint_control[6] = {0};
    unsigned char type_of_control[6] = {0};
    signed char position_setpoint[6] = {0};
    signed char torque_setpoint[6] = {0};
    signed char stiffness_setpoint[6] = {0};
    signed char min_angles[6] = {-25, 0, -25, -25, 0, -25};
    signed char max_angles[6] = {100, 100, 25, 100, 100, 25};
    unsigned char percentage_assistance[6] = {100, 100, 100, 100, 100, 100};
    unsigned char start_stop_CAN[6] = {0};
    unsigned char robot_command[6] = {0};
    unsigned char communication_mode[6] = {0};
    double duty_cycle[6] = {0};
    unsigned char trigger_out = 0;

    //Record data
    std::string base_file_name = "h3_data";
    int start_stop_recording = 0;
    double recording_time_sec = 0;
    bool timed_recording = false;

  }; //struct

  /**
 * @brief Class for handle the CAN communication with the H3.
 * Reading and decoding of CAN messages.
 * Coding and writing of CAN messages.
 */
  class H3Handle
  {
  public:
    H3Handle() {}
    ~H3Handle();

    /**
     * @brief Start CAN communication.
     * 
     * @param channel The number of the CAN channel connected to the H3.
     * @param last_CAN_message Last CAN message in the message array.
     * @return int CAN error status. 
     */
    int CANBegin(unsigned int channel, unsigned int last_CAN_message, int external_CAN_mode);

    /**
     * @brief A pointer to H3 States storage.
     * 
     * @return const H3StateHW* 
     */
    const H3StateHW *readH3();

    /**
     * @brief Write H3 command struct storange.
     * 
     * @param cmd 
     * @return true 
     * @return false 
     */
    bool writeH3(H3CommandHW cmd);

    /**
     * @brief Write PID gains to internal controller
     * 
     * @param control_type Internal control type
     * @param kp  Proportional gain
     * @param ki  Integral gain
     * @param kd  Derivative gain
     * @param i_clamp Integral clamp
     * @return true 
     * @return false 
     */
    bool writeControllersGains(const char control_type, const std::vector<double> &kp, const std::vector<double> &ki, const std::vector<double> &kd, const std::vector<double> &i_clamp);
    
    /**
     * @brief Read PID gains from internal controller
     * 
     * @param control_type Internal control type
     * @param kp  Proportional gain
     * @param ki  Integral gain
     * @param kd  Derivative gain
     * @param i_clamp Integral clamp
     * @return true 
     * @return false
     */
    bool readControllersGains(const char control_type,  std::vector<double> &kp, std::vector<double> &ki,  std::vector<double> &kd, std::vector<double> &i_clamp);
    
    /**
     * @brief Starts the update trhead.
     * 
     */
    void Start();

    /**
     * @brief Stops the update thread.
     * 
     */
    void Stop();

    /**
     * @brief Indicate if the update thread is running.
     * 
     * @return true 
     * @return false 
     */
    bool is_reading() const;

    /**
     * @brief Set the Recording object
     * 
     * @param recording_convention 
     * @param recording_mode 
     */
    void setRecording(int recording_convention, int recording_mode);

    /**
     * @brief Gets a CAN error status description.
     * 
     * @param status_code View CAN interface driver library for most details.
     * @param language_code Lenguage ID
     * @param error_text A Pointer null termined char array.
     */
    void getCANErrorText(TPCANStatus status_code, char language_code, char *error_text);

  private:
    /**
     * @brief Decodes received CAN messages and stores them in \ref the H3StateHW data structure.
     * 
     * @param message 
     * @param state
     */
    void decodeHighResolutionData(const TPCANMsg &message, H3StateHW *state);

    /**
     * @brief Decodes received CAN messages and stores them in \ref the H3StateHW data structure.
     * 
     * @param message 
     * @param state 
     */
    void decodeLowResolutionData(const TPCANMsg &message, H3StateHW *state);

    /**
     * @brief Update thread: read, decode, save, encode and write CAN messages.
     * 
     * @return int CAN error status.
     */
    void update();

    /**
   * @brief Write CAN message to H3 Main Controller. 
   * 
   * @param CAN_id CAN identifier acepted by H3 Main Controller.
   * @return int CAN error status.
   */
    int CANWrite(h3_definition::CommandCANFrame CAN_ID, H3CommandHW &command);

    /**
     * @brief Set the High Resolution Mode in Main Controller. 
     * In this mode, the Main Controller transmit each sensor data using 16-bits. 
     * 
     * @return int 
     */
    int setHighResolutionMode();

    /**
     * @brief Set the Low Resolution Mode in Main Controller.
     * In this mode, the Main Controller transmit each sensor data using 8-bits. 
     * @return int 
     */
    int setLowResolutionMode();

    H3StateHW h3_state_;      //storing State data
    H3StateHW h3_state_copy_; //for data interchange
    H3StateHW *h3_state_ptr_ = &h3_state_;
    H3StateHW *h3_state_copy_ptr_ = &h3_state_copy_;
    bool keep_reading_ = false;
    bool data_available_ = false;
    bool is_reading_ = false;

    H3CommandHW h3_command_; //storing Command data
    H3CommandHW h3_command_copy_;
    H3CommandHW *h3_command_ptr_ = &h3_command_;
    H3CommandHW *h3_command_copy_ptr_ = &h3_command_copy_;
    bool command_available_ = false;

    int recording_convention_ = 0;
    int recording_mode_ = 0;

    TPCANStatus status_;
    TPCANHandle channel_;
    bool read_CAN_running_;
    int external_CAN_mode_;

    std::thread thread_;
    std::mutex mutex_;
    unsigned int last_message_;

  }; // class
} // namespace h3_hardware_interface