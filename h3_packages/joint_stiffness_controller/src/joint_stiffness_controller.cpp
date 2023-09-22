///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021, Technaid S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Technaid S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include "joint_stiffness_controller/joint_stiffness_controller.h"

namespace joint_stiffness_controller
{
    bool JointStiffnessController::init(hardware_interface::StiffnessJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        // Get named robot from parameter server
        std::string joint_name;
        if (!controller_nh.getParam("joint", joint_name))
        {
            ROS_ERROR("No joint given (namespace: %s)", controller_nh.getNamespace().c_str());
            return false;
        }
        // get a handle for command the robot
        joint_handle_ = hw->getHandle(joint_name);

        sub_ = controller_nh.subscribe("command", 1, &JointStiffnessController::command, this);
        return true;
    }

    void JointStiffnessController::starting(const ros::Time &time)
    {
        //initial state
        comand_.position_cmd = joint_handle_.getPosition();
        comand_.stiffness_cmd = 0;
        command_buffer_.initRT(comand_);
    }
    void JointStiffnessController::update(const ros::Time &time, const ros::Duration &period)
    {
        //set command data
        comand_ = *command_buffer_.readFromNonRT();
        joint_handle_.setCommand(comand_.position_cmd, comand_.stiffness_cmd);
    }
    void JointStiffnessController::stopping(const ros::Time &time)
    {
    }

    void JointStiffnessController::command(const std_msgs::Float64MultiArray::ConstPtr &cmd)
    {
        Command cmd_buf;
        cmd_buf.position_cmd = cmd->data[0];
        cmd_buf.stiffness_cmd = cmd->data[1];
        command_buffer_.writeFromNonRT(cmd_buf);
    }

    PLUGINLIB_EXPORT_CLASS(joint_stiffness_controller::JointStiffnessController, controller_interface::ControllerBase);
} // namespace joint_stiffness_controller