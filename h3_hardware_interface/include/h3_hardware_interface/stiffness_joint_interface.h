///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012,Technaid S.L., hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Technaid S.L, hiDOF, Inc. nor the names of its
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

/// \author Wim Meeussen


/**
 * @brief Modified version of the JointHandle class in which an additional command (stiff_cmd) is added.
 * 
 */

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{ /** \brief A handle used to read and command a single joint. */
    class StiffnessJointHandle : public JointStateHandle
    {
    public:
        StiffnessJointHandle() = default;

        /**
   * \param js This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   */
        StiffnessJointHandle(const JointStateHandle &js, double *pos_cmd, double *stiff_cmd)
            : JointStateHandle(js), pos_cmd_(pos_cmd), stiff_cmd_(stiff_cmd)
        {
            if (!pos_cmd_||!stiff_cmd_)
            {
                throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
            }
        }

        void setCommand(double position_cmd, double stiffness_cmd)
        {
            assert(pos_cmd_);
            *pos_cmd_ = position_cmd;
            assert(stiff_cmd_);
            *stiff_cmd_ = stiffness_cmd;
        }
        double getPositionCommand() const
        {
            assert(pos_cmd_);
            return *pos_cmd_;
        }
        double getStiffnessCommand() const
        {
            assert(stiff_cmd_);
            return *stiff_cmd_;
        }
        const double *getPositionCommandPtr() const
        {
            assert(pos_cmd_);
            return pos_cmd_;
        }
        const double *getStiffnessCommandPtr() const
        {
            assert(stiff_cmd_);
            return stiff_cmd_;
        }

    private:
        double *pos_cmd_ = {nullptr};
        double *stiff_cmd_ = {nullptr};
    };
    
    class StiffnessJointInterface : public HardwareResourceManager<StiffnessJointHandle, ClaimResources>
    {
    };
} // namespace hardware_interface