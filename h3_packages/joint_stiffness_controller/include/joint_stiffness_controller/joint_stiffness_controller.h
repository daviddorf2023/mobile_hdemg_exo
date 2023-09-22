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

#pragma once

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller.h>
#include <h3_hardware_interface/stiffness_joint_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64MultiArray.h>

namespace joint_stiffness_controller
{

    class JointStiffnessController : public controller_interface::Controller<hardware_interface::StiffnessJointInterface>
    {
    public:
        JointStiffnessController(/* args */){}
        ~JointStiffnessController(){}

        struct Command
        {
            Command() {}
            double position_cmd;
            double stiffness_cmd;
        };
        //Controller functions
        bool init(hardware_interface::StiffnessJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
        void starting(const ros::Time &time);
        void update(const ros::Time &time, const ros::Duration &period);
        void stopping(const ros::Time &time);

        //ROS subscriber callback function
        void command(const std_msgs::Float64MultiArray::ConstPtr &cmd);

    private:
        Command comand_;
        realtime_tools::RealtimeBuffer<Command> command_buffer_;
        hardware_interface::StiffnessJointHandle joint_handle_;
        ros::Subscriber sub_;
    };

} // namespace joint_stiffness_controller