/*LGPL-2.1*/
/*
 * Copyright (C) 2021  Technaid S.L. <www.technaid.com>
 *.
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

#include "h3_hardware_interface/h3_handle.h"
#include "h3_hardware_interface/h3_logger.h"

namespace h3_hardware_interface
{
  using namespace h3_definition;

  /* */
  H3Handle::~H3Handle()
  {
    while (keep_reading_)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      keep_reading_ = false;
    }

    if (thread_.joinable())
    {
      thread_.join();
      CAN_Uninitialize(channel_);
    }
  }

  /* */
  int H3Handle::CANBegin(unsigned int channel, unsigned int last_CAN_message, int external_CAN_mode)
  {
    last_message_ = last_CAN_message;
    external_CAN_mode_ = external_CAN_mode;
    channel_ = channel;
    CAN_Initialize(channel_, PCAN_BAUD_1M, 0, 0, 0);
    if (external_CAN_mode_ == ExternalCANMode::CANTransmissionHighResolution)
    {
      return setHighResolutionMode();
    }
    return setLowResolutionMode();
  }

  void H3Handle::setRecording(int recording_convention, int recording_mode)
  {
    recording_convention_ = recording_convention;
    recording_mode_ = recording_mode;
  }

  void H3Handle::getCANErrorText(TPCANStatus status_code, char language_code, char *error_text)
  {
    CAN_GetErrorText(status_code, language_code, error_text);
  }

  /*
  *Read-Decode-Record-Encode-Write*/
  /** This function is executed in a thread started with the start()
  * function and stopped by the stop() function.
  * Its function is to read, decode, record, encode and write on the exo-H3.
  * and implements shared memory with main thread.
 */
  void H3Handle::update()
  {

    read_CAN_running_ = true;
    H3StateHW state_hw;
    H3CommandHW command_hw;
    H3CommandHW command_k_1;
    bool cmd_available = false;
    int time_out_ms = 30;
    int time_out_last_msg_ms = 20;
    H3Logger logger;

    auto t0 = std::chrono::high_resolution_clock::now();

    while (keep_reading_)
    {
      auto t1 = std::chrono::high_resolution_clock::now();
      auto t2 = std::chrono::high_resolution_clock::now();
      bool time_out = false;
      TPCANMsg message;

      /* Wait here until a CAN message arrives or a time_out occurs. */
      while ((CAN_Read(channel_, &message, NULL)) == PCAN_ERROR_QRCVEMPTY && ((t2 - t1) < std::chrono::milliseconds(time_out_ms)))
      {
        t2 = std::chrono::high_resolution_clock::now();
        if ((t2 - t1) >= std::chrono::milliseconds(time_out_ms))
        {
          message.ID = 0;
          state_hw.connection_status = 0;
          time_out = true;
        }
        else
        {
          time_out = false;
        }
      }
      /* Check the last message arrived*/
      if ((t1 - t0) >= std::chrono::milliseconds(time_out_last_msg_ms))
      {
        state_hw.connection_status = 0;
        /* Try set the CAN communication mode for recovery the connection. */
        if (external_CAN_mode_ == ExternalCANMode::CANTransmissionHighResolution)
        {
          setHighResolutionMode();
        }
        if (external_CAN_mode_ == ExternalCANMode::CANTransmissionLowResolution)
        {
          setLowResolutionMode();
        }
      }
      if (message.ID == last_message_)
      {
        t0 = std::chrono::high_resolution_clock::now();
        state_hw.connection_status = 1;
      }

      /*Data Decoding*/
      if (external_CAN_mode_ == ExternalCANMode::CANTransmissionHighResolution)
      {
        decodeHighResolutionData(message, &state_hw);
      }
      if (external_CAN_mode_ == ExternalCANMode::CANTransmissionLowResolution)
      {
        decodeLowResolutionData(message, &state_hw);
      }

      if (message.ID == last_message_ || time_out)
      {
        /* Data recording logic. */
        logger.recordStateHW(&state_hw, &command_hw, recording_convention_, recording_mode_, command_hw.base_file_name, command_hw.start_stop_recording, command_hw.recording_time_sec, command_hw.timed_recording);
        state_hw.record_status = logger.getRecordingStatus();

        /* locks shared memory and notifies that there is new data.*/
        while (!mutex_.try_lock())
        {
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        *h3_state_ptr_ = state_hw;
        data_available_ = true;
        mutex_.unlock();

        /* Wait 5 ms before write*/
        //std::this_thread::sleep_for(std::chrono::microseconds(5000));

        /* Lock shared command data*/
        while (!mutex_.try_lock())
        {
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        if (command_available_)
        {
          H3CommandHW *temp;
          temp = h3_command_ptr_;
          h3_command_ptr_ = h3_command_copy_ptr_;
          h3_command_copy_ptr_ = h3_command_ptr_;
          command_available_ = false;
          cmd_available = true;
        }
        command_hw = *h3_command_ptr_;
        mutex_.unlock();

        /* Write message*/
        /** Write messages according to the type of control selected 
        * Write messages only when its values change and that do not depend on the type of control selected
        * Write messages that do not change but indicate an event, for example, a trigger message.
         */
        bool write_position = false;
        bool write_stiffness = false;
        bool write_torque = false;
        bool write_duty_cycle = false;
        bool write_task = false;
        int task_mode_counter = 0;

        bool write_control_type = false;
        bool write_assistance = false;
        bool write_max_angles = false;
        bool write_min_angles = false;
        bool trigger_out = false;

        for (int i = 0; i < 6; ++i)
        {
          if (command_hw.type_of_control[i] == SelectControlType::MotorDisable)
          {
            /* To disable the motor, it is not necessary to send any setpoint */
            state_hw.control_type[i] = SelectControlType::MotorDisable;
          }
          if (command_hw.type_of_control[i] == SelectControlType::PositionControl)
          {
            state_hw.control_type[i] = SelectControlType::PositionControl;
            write_position = true;
          }
          if (command_hw.type_of_control[i] == SelectControlType::StiffnessControl)
          {
            state_hw.control_type[i] = SelectControlType::StiffnessControl;
            write_position = true;
            write_stiffness = true;
          }
          if (command_hw.type_of_control[i] == SelectControlType::TorqueControl)
          {
            state_hw.control_type[i] = SelectControlType::TorqueControl;
            write_torque = true;
          }
          if (command_hw.type_of_control[i] == SelectControlType::OpenLoop)
          {
            state_hw.control_type[i] = SelectControlType::OpenLoop;
            write_duty_cycle = true;
          }
          if (command_hw.type_of_control[i] == SelectControlType::TaskController)
          {
            state_hw.control_type[i] = SelectControlType::TaskController;
            task_mode_counter++;
            write_task = true;
          }
          if (command_hw.type_of_control[i] == SelectControlType::MotorStopped)
          {
            state_hw.control_type[i] = SelectControlType::MotorStopped;
            command_hw.position_setpoint[i] = state_hw.joint_angle[i];
            write_position = true;
          }

          /*A change in control type value*/
          if (command_hw.type_of_control[i] != command_k_1.type_of_control[i])
          {
            command_k_1.type_of_control[i] = command_hw.type_of_control[i];
            write_control_type = true;
          }
          /*A change in assistance value */
          if (command_hw.percentage_assistance[i] != command_k_1.percentage_assistance[i])
          {
            command_k_1.percentage_assistance[i] = command_hw.percentage_assistance[i]; //without write verification
            write_assistance = true;
          }
          /*A change in max angles value */
          if (command_hw.max_angles[i] != command_k_1.max_angles[i])
          {
            command_k_1.max_angles[i] = command_hw.max_angles[i]; //without write verification
            write_max_angles = true;
          }
          /*A change in min angles value */
          if (command_hw.min_angles[i] != command_k_1.min_angles[i])
          {
            command_k_1.min_angles[i] = command_hw.min_angles[i]; //without write verification
            write_min_angles = true;
          }
          /*Trigger logic */
          if (command_hw.trigger_out != command_k_1.trigger_out)
          {
            command_k_1.trigger_out = command_hw.trigger_out;
            trigger_out = true;
          }
          else
          {
            if (command_hw.trigger_out == TriggerOutputMode::TriggerPulse)
            {
              trigger_out = true;
            }
          }
        }

        /* Write Control Type. */
        if (write_control_type && cmd_available)
        {
          CANWrite(CommandCANFrame::ControlType, command_hw);
          //std::cout << "Write control type to Hw " << std::endl;
        }

        /* Write Torque setpoint. */
        if (write_position && cmd_available)
        {
          CANWrite(CommandCANFrame::PositionSetpoint, command_hw);
          //std::cout << "Write position to Hw " << std::endl;
        }

        /* Write stiffness setpoint. */
        if (write_stiffness && cmd_available)
        {
          //std::this_thread::sleep_for(std::chrono::milliseconds(2)); //Wait to internal loop(1 ms) process the data
          CANWrite(CommandCANFrame::StiffnessSetpoint, command_hw);
          //std::cout << "Write stiffness to Hw " << std::endl;
        }

        /* Write Torque setpoint. */
        if (write_torque && cmd_available)
        {
          CANWrite(CommandCANFrame::TorqueSetpoint, command_hw);
          //std::cout << "Write Torque to Hw " << std::endl;
        }

        /* Write PWM duty_cycle */
        if (write_duty_cycle && cmd_available)
        {
          CANWrite(CommandCANFrame::RightLegDutyCycle, command_hw);
          CANWrite(CommandCANFrame::LeftLegDutyCycle, command_hw);
          //std::cout << "Write PWM duty cycle to Hw " << std::endl;
        }

        /* Write Percentage of Assistance. */
        if (write_assistance && cmd_available)
        {
          CANWrite(CommandCANFrame::PerecentageOfAssistance, command_hw);
          //std::cout << "Write Percentage of Assistance to Hw " << std::endl;
        }

        /* Write max_angles. */
        if (write_max_angles && cmd_available)
        {
          CANWrite(CommandCANFrame::MaxAnglesAccepted, command_hw);
          //std::cout << "Write Max angles to Hw " << std::endl;
        }

        /* Write min_angles. */
        if (write_min_angles && cmd_available)
        {
          CANWrite(CommandCANFrame::MinAnglesAccepted, command_hw);
          //std::cout << "Write Min angles to Hw " << std::endl;
        }

        /* Write Robot Task */
        if (write_task && cmd_available && task_mode_counter == 6)
        {

          if (command_hw.robot_command[0] == 0 && !(command_hw.robot_command[1] == 0 && command_k_1.robot_command[1] == 0))
          {
            CANWrite(CommandCANFrame::ExoCommand, command_hw); //avoid 0 0 secuence.
            //std::cout << "Write Robot Task to Hw " << std::endl;
          }
          else if(command_hw.robot_command[0] == 1)
          {
            CANWrite(CommandCANFrame::ExoCommand, command_hw);
            //std::cout << "Write Robot Task to Hw " << std::endl;
          }
          command_k_1.robot_command[1] = command_hw.robot_command[1];
        }
        /* Trigger */
        if (trigger_out && cmd_available)
        {
          command_hw.communication_mode[0] = external_CAN_mode_;
          command_hw.communication_mode[1] = command_hw.trigger_out;
          CANWrite(CommandCANFrame::CommunicationMode, command_hw);
          //std::cout << "Write Trigger to Hw " << std::endl;
        }

        cmd_available = false;
      }
    }
    read_CAN_running_ = false;
  }

  bool H3Handle::writeControllersGains(const char control_type, const std::vector<double> &kp, const std::vector<double> &ki, const std::vector<double> &kd, const std::vector<double> &i_clamp)
  {
    TPCANMsg message;
    message.LEN = 8;
    message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    message.ID = CommandCANFrame::Configuration;

    for (int i = 0; i < 2; ++i) // Write parameter to right leg and left leg
    {
      for (int j = 0; j < 4; ++j) //gains
      {
        if (i == 0) // Right leg
        {
          message.DATA[0] = MatrixIdentifiers::M_3_16_W;
        }
        if (i == 1) // Left leg
        {
          message.DATA[0] = MatrixIdentifiers::M_5_16_W;
        }
        if (control_type == SelectControlType::PositionControl)
        {
          message.DATA[1] = j + 0;
        }
        else if (control_type == SelectControlType::StiffnessControl)
        {
          message.DATA[1] = j + 4;
        }
        else if (control_type == SelectControlType::TorqueControl)
        {
          message.DATA[1] = j + 8;
        }
        else
        {
          return false;
        }

        for (int k = 0; k < 3; ++k) // Signal number
        {
          if (j == 0) //kp_constants
          {
            if (i == 0) // Right leg
            {
              uint16_t constants_aux = (uint16_t)(kp[k] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
            if (i == 1)
            {
              uint16_t constants_aux = (uint16_t)(kp[k + 3] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
          }
          if (j == 1) //ki_constants
          {
            if (i == 0) // Right leg
            {
              uint16_t constants_aux = (uint16_t)(ki[k] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
            if (i == 1)
            {
              uint16_t constants_aux = (uint16_t)(ki[k + 3] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
          }
          if (j == 2) //kd_constants
          {
            if (i == 0) // Right leg
            {
              uint16_t constants_aux = (uint16_t)(kd[k] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
            if (i == 1)
            {
              uint16_t constants_aux = (uint16_t)(kd[k + 3] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
          }
          if (j == 3) //i_clamp constants
          {
            if (i == 0) // Right leg
            {
              uint16_t constants_aux = (uint16_t)(i_clamp[k] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
            if (i == 1)
            {
              uint16_t constants_aux = (uint16_t)(i_clamp[k + 3] * 100.0);
              message.DATA[2 * k + 2] = constants_aux & 0x00FF;
              message.DATA[2 * k + 3] = (constants_aux >> 8) & 0x00FF;
            }
          }
        }
        if (CAN_Write(channel_, &message))
        {
          return false;
        }
        CAN_Write(channel_, &message);
      }
    }
    return true;
  }

  /**/
  bool H3Handle::readControllersGains(const char control_type, std::vector<double> &kp, std::vector<double> &ki, std::vector<double> &kd, std::vector<double> &i_clamp)
  {

    TPCANMsg message;
    message.LEN = 8;
    message.MSGTYPE = PCAN_MESSAGE_STANDARD;

    /* Stop Continuous CAN transmitions*/
    message.ID = CommandCANFrame::CommunicationMode;
    message.DATA[0] = 0;
    CAN_Write(channel_, &message);

    message.ID = CommandCANFrame::Configuration;
    int msg_conf_cnt = 0; //verified Configuration msg received

    for (int i = 0; i < 2; ++i) // Write parameter to right leg and left leg
    {
      for (int j = 0; j < 4; ++j) //gains kp, ki, kd, i_clamp
      {
        if (i == 0) // Right leg
        {
          message.DATA[0] = MatrixIdentifiers::M_2_16_R;
        }
        if (i == 1) // Left leg
        {
          message.DATA[0] = MatrixIdentifiers::M_4_16_R;
        }
        if (control_type == SelectControlType::PositionControl)
        {
          message.DATA[1] = j + 0;
        }
        else if (control_type == SelectControlType::StiffnessControl)
        {
          message.DATA[1] = j + 4;
        }
        else if (control_type == SelectControlType::TorqueControl)
        {
          message.DATA[1] = j + 8;
        }
        else
        {
          return false;
        }

        /*Request gain */
        if (CAN_Write(channel_, &message))
        {
          return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Wait for MC response
        TPCANMsg message2;
        uint16_t *t16;
        /* Wait here until a CAN message arrives or a time_out occurs. */
        while ((CAN_Read(channel_, &message2, NULL)) != PCAN_ERROR_QRCVEMPTY)
        {
          if (message2.ID == StatusCANFrame::ConfigurationResponse)
          {
            msg_conf_cnt++;
            for (int k = 0; k < 3; ++k) // Signal number
            {
              if (j == 0) // kp
              {
                if (i == 0 && message2.DATA[0] == MatrixIdentifiers::M_2_16_R) // Right leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  kp.push_back((*t16) / 100.0);
                }
                else if (i == 1 && message2.DATA[0] == MatrixIdentifiers::M_4_16_R) // Left leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  kp.push_back((*t16) / 100.0);
                }
                else
                {
                  return false;
                }
              }
              if (j == 1)
              {
                if (i == 0 && message2.DATA[0] == MatrixIdentifiers::M_2_16_R) // Right leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  ki.push_back((*t16) / 100.0);
                }
                else if (i == 1 && message2.DATA[0] == MatrixIdentifiers::M_4_16_R) // Left leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  ki.push_back((*t16) / 100.0);
                }
                else
                {
                  return false;
                }
              }
              if (j == 2)
              {
                if (i == 0 && message2.DATA[0] == MatrixIdentifiers::M_2_16_R) // Right leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  kd.push_back((*t16) / 100.0);
                }
                else if (i == 1 && message2.DATA[0] == MatrixIdentifiers::M_4_16_R) // Left leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  kd.push_back((*t16) / 100.0);
                }
                else
                {
                  return false;
                }
              }
              if (j == 3)
              {
                if (i == 0 && message2.DATA[0] == MatrixIdentifiers::M_2_16_R) // Right leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  i_clamp.push_back((*t16) / 100.0);
                }
                else if (i == 1 && message2.DATA[0] == MatrixIdentifiers::M_4_16_R) // Left leg
                {
                  t16 = (uint16_t *)&message2.DATA[2 * k + 2];
                  i_clamp.push_back((*t16) / 100.0);
                }
                else
                {
                  return false;
                }
              }
            }
          }
        }
      }
    }
    /* Start continuous CAN transmitions*/
    message.ID = CommandCANFrame::CommunicationMode;
    message.DATA[0] = external_CAN_mode_;
    CAN_Write(channel_, &message);
    if (msg_conf_cnt < 8)
    {
      return false;
    }
    return true;
  }

  void H3Handle::Start()
  {
    if (!keep_reading_)
    {
      keep_reading_ = true;
      thread_ = std::thread(&H3Handle::update, this);
    }
  }

  void H3Handle::Stop()
  {
    if (keep_reading_)
    {
      keep_reading_ = false;
    }
  }

  void H3Handle::decodeHighResolutionData(const TPCANMsg &message, H3StateHW *state)
  {
    switch (message.ID)
    {
      int16_t *t16;
    case StatusCANFrame::RightLegAngle:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->joint_angle[i] = (*t16) / 10.0;
      }
      break;
    case StatusCANFrame::LeftLegAngle:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->joint_angle[i + 3] = (*t16) / 10.0;
      }
      break;
    case StatusCANFrame::RightLegVelocity:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->joint_velocity[i] = (*t16) / 10.0;
      }
      break;
    case StatusCANFrame::LeftLegVelocity:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->joint_velocity[i + 3] = (*t16) / 10.0;
      }
      break;
    case StatusCANFrame::RightLegTorque:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->joint_torque[i] = (*t16) / 10.0;
      }
      break;
    case StatusCANFrame::LeftLegTorque:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->joint_torque[i + 3] = (*t16) / 10.0;
      }
      break;
    case StatusCANFrame::RightLegMotorTorque:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->motor_torque[i] = (*t16) / 10.0;
      }

      break;
    case StatusCANFrame::LeftLegMotorTorque:
      for (int i = 0; i < 3; i++)
      {
        t16 = (int16_t *)&message.DATA[2 * i];
        state->motor_torque[i + 3] = (*t16) / 10.0;
      }
      break;
    case StatusCANFrame::JointFootSwitch2:
      state->right_heel = (unsigned char)message.DATA[0];
      state->right_toe = (unsigned char)message.DATA[1];
      state->left_heel = (unsigned char)message.DATA[2];
      state->left_toe = (unsigned char)message.DATA[3];
      state->battery_voltage = message.DATA[4] / 10.0;
      state->trigger_input = message.DATA[5];
      break;
    case StatusCANFrame::ExoStateFrame2:
      state->exo_state = (unsigned char)message.DATA[0];
      state->commandApp = (unsigned char)message.DATA[1];
      state->trigger_counter = (unsigned char)message.DATA[2];
      state->exo_run_time_min = (unsigned char)message.DATA[3];
      state->exo_run_time_sec = (unsigned char)message.DATA[4];
      state->exo_run_time_csec = (unsigned char)message.DATA[5];
      break;
    default:
      break;
    }
  }

  void H3Handle::decodeLowResolutionData(const TPCANMsg &message, H3StateHW *state)
  {
    switch (message.ID)
    {
      signed char aux;
    case StatusCANFrame::JointAngle:
      for (int i = 0; i < 6; ++i)
      {
        aux = (signed char)message.DATA[i];
        state->joint_angle[i] = static_cast<double>(aux);
      }
      break;
    case StatusCANFrame::JointTorque:
      for (int i = 0; i < 6; ++i)
      {
        aux = (signed char)message.DATA[i];
        state->joint_torque[i] = static_cast<double>(aux);
      }
      break;
    case StatusCANFrame::JointMotorTorque:
      for (int i = 0; i < 6; ++i)
      {
        aux = (signed char)message.DATA[i];
        state->motor_torque[i] = static_cast<double>(aux);
      }
      break;
    case StatusCANFrame::JointFootSwitch:
      state->right_heel = (unsigned char)message.DATA[0];
      state->right_toe = (unsigned char)message.DATA[1];
      state->left_heel = (unsigned char)message.DATA[2];
      state->left_toe = (unsigned char)message.DATA[3];
      state->battery_voltage = message.DATA[4] / 10.0;
      state->trigger_input = message.DATA[5];
      break;
    case StatusCANFrame::ExoStateFrame:
      state->exo_state = (unsigned char)message.DATA[0];
      state->commandApp = (unsigned char)message.DATA[1];
      state->trigger_counter = (unsigned char)message.DATA[2];
      state->exo_run_time_min = (unsigned char)message.DATA[3];
      state->exo_run_time_sec = (unsigned char)message.DATA[4];
      state->exo_run_time_csec = (unsigned char)message.DATA[5];
      break;
    default:
      break;
    }
  }
  /* */
  int H3Handle::CANWrite(h3_definition::CommandCANFrame CAN_ID, H3CommandHW &command)
  {
    TPCANMsg message;
    message.LEN = 6;
    message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    message.ID = static_cast<int>(CAN_ID);

    switch (CAN_ID)
    {
    case CommandCANFrame::ControlType:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.type_of_control[i];
      break;

    case CommandCANFrame::PositionSetpoint:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.position_setpoint[i];
      break;

    case CommandCANFrame::JointControl:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.joint_control[i];
      break;

    case CommandCANFrame::TorqueSetpoint:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.torque_setpoint[i];
      break;

    case CommandCANFrame::StiffnessSetpoint:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.stiffness_setpoint[i];
      break;

    case CommandCANFrame::MinAnglesAccepted:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.min_angles[i];
      break;

    case CommandCANFrame::MaxAnglesAccepted:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.max_angles[i];
      break;

    case CommandCANFrame::PerecentageOfAssistance:

      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.percentage_assistance[i];
      break;

    case CommandCANFrame::ExoCommand:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.robot_command[i];
      break;

    case CommandCANFrame::RightLegDutyCycle:

      for (int i = 0; i < 3; i++)
      {
        int16_t duty_cycle_aux = (int16_t)(command.duty_cycle[i] * 36);
        message.DATA[2 * i] = duty_cycle_aux & 0x00FF;
        message.DATA[2 * i + 1] = (duty_cycle_aux >> 8) & 0x00FF;
      }
      break;

    case CommandCANFrame::LeftLegDutyCycle:
      for (int i = 3; i < 6; i++)
      {
        int16_t duty_cycle_aux = (int16_t)(command.duty_cycle[i] * 36);
        message.DATA[2 * i - 6] = duty_cycle_aux & 0x00FF;
        message.DATA[2 * i - 5] = (duty_cycle_aux >> 8) & 0x00FF;
      }
      break;

    case CommandCANFrame::CommunicationMode:
      for (int i = 0; i < 6; i++)
        message.DATA[i] = command.communication_mode[i];
      break;

    default:
      return -1;
      break;
    }
    return CAN_Write(channel_, &message);
  }

  int H3Handle::setHighResolutionMode()
  {
    TPCANMsg message;
    message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    message.ID = CommandCANFrame::CommunicationMode;
    message.LEN = 6;
    message.DATA[0] = ExternalCANMode::CANTransmissionHighResolution;
    return CAN_Write(channel_, &message);
  }
  int H3Handle::setLowResolutionMode()
  {
    TPCANMsg message;
    message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    message.ID = CommandCANFrame::CommunicationMode;
    message.LEN = 6;
    message.DATA[0] = ExternalCANMode::CANTransmissionLowResolution;
    return CAN_Write(channel_, &message);
  }

  const H3StateHW *H3Handle::readH3()
  {
    /* This function implements an exchange of pointers between the two data structures*/
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock())
    {
      if (data_available_)
      {
        H3StateHW *temp;
        temp = h3_state_copy_ptr_;
        h3_state_copy_ptr_ = h3_state_ptr_;
        h3_state_ptr_ = temp;
        data_available_ = false;
      }
    }
    return h3_state_copy_ptr_;
  }
  /**/
  bool H3Handle::writeH3(H3CommandHW cmd)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock())
    {
      *h3_command_copy_ptr_ = cmd;
      command_available_ = true;
      return true;
    }
    return false;
  }

} // namespace h3_hardware_interface
