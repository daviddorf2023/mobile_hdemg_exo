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

namespace h3_definition
{
    /** \brief Commands accepted by Task Controller */
    enum TaskControllerCommand
    {
        StopGait = 0,
        WalkSpeed1 = 1,
        WalkSpeed2 = 2,
        WalkSpeed3 = 3,
        WalkSpeed4 = 4,
        WalkSpeed5 = 5,
        WalkSpeed6 = 6,
        WalkSpeed7 = 7,
        WalkSpeed8 = 8,
        WalkSpeed9 = 9,
        WalkSpeed10 = 10,
        JointsPassive = 11,
        JointsCompliants = 12,
        EnableSound = 19,
        DisableSound = 20,
        StandUp = 21,
        StitDown = 22,
        PerformRightStep = 23,
        PerformLeftStep = 24,
        RightLegMaxAssistance_10 = 31,
        RightLegMaxAssistance_20 = 32,
        RightLegMaxAssistance_30 = 33,
        RightLegMaxAssistance_40 = 34,
        RightLegMaxAssistance_50 = 35,
        RightLegMaxAssistance_60 = 36,
        RightLegMaxAssistance_70 = 37,
        RightLegMaxAssistance_80 = 38,
        RightLegMaxAssistance_90 = 39,
        RightLegMaxAssistance_100 = 40,
        LeftLegMaxAssistance_10 = 41,
        LeftLegMaxAssistance_20 = 42,
        LeftLegMaxAssistance_30 = 43,
        LeftLegMaxAssistance_40 = 44,
        LefttLegMaxAssistance_50 = 45,
        LeftLegMaxAssistance_60 = 46,
        LeftLegMaxAssistance_70 = 47,
        LeftLegMaxAssistance_80 = 48,
        LeftLegMaxAssistance_90 = 49,
        LeftLegMaxAssistance_100 = 50,
        EnableBluetoothAppCommand = 52,
    };

    enum TaskControllerState
    {
        StandingUP = 0,
        WalkingSpeed1 = 1,
        WalkingSpeed2 = 2,
        WalkingSpeed3 = 3,
        WalkingSpeed4 = 4,
        WalkingSpeed5 = 5,
        WalkingSpeed6 = 6,
        WalkingSpeed7 = 7,
        WalkingSpeed8 = 8,
        WalkingSpeed9 = 9,
        JointsInPassive = 11,
        JointsInCompliant = 12,
        IsStopppingWalk = 13,
        IsPerformingStandUp = 14,
        IsPerforminSitDown = 15,
        SeatedDown = 16,
        PerformingLeftStep = 17,  
        PerformingRightStep = 18, 
        AllJointsBlocked = 19,
        RightHipFailure = 21,
        RightKneeFailure = 22,
        RightAnkleFailure = 23,
        LeftHipFailure = 24,
        LeftKneeFailure = 25,
        LeftAnkleFailure = 26,
    };

    /** \brief //CAN frame identifiers sent to H3 */
    enum CommandCANFrame
    {
        JointControl = 70,
        ControlType = 71,
        PositionSetpoint = 72,
        TorqueSetpoint = 73,
        StiffnessSetpoint = 74,
        MinAnglesAccepted = 75,
        PerecentageOfAssistance = 76,
        MaxAnglesAccepted = 80,
        ExoCommand = 81,
        Configuration = 82,
        RightLegDutyCycle = 83,
        LeftLegDutyCycle = 84,
        CommunicationMode = 85,
    };

    /** \brief CAN frame identifiers recived from H3 */
    enum StatusCANFrame
    {
        ConfigurationResponse = 100,
        JointAngle = 110,
        JointTorque = 120,
        JointFootSwitch = 130,
        JointMotorTorque = 140,
        ExoStateFrame = 150,
        RightLegAngle = 50,
        LeftLegAngle = 51,
        RightLegVelocity = 52,
        LeftLegVelocity = 53,
        RightLegTorque = 54,
        LeftLegTorque = 55,
        RightLegMotorTorque = 56,
        LeftLegMotorTorque = 57,
        JointFootSwitch2 = 58,
        ExoStateFrame2 = 59,
    };

    /** \brief H3 Control modes */
    enum SelectControlType
    {
        MotorDisable = 0, // No voltage applied to motors.
        PositionControl = 1,
        StiffnessControl = 2,
        TorqueControl = 3,
        MotorStopped = 5,
        OpenLoop = 6,
        TaskController = 7, // The task controller only works when all joints are in this mode.
    };

    /** \brief Number that identifies each joint, 
     * take it into account when sending \ref CommandCANFrame::JointControl 
     */
    enum class JointsID
    {
        RightHip = 1,
        RightKnee = 2,
        RightAnkle = 3,
        LeftHip = 4,
        LeftKnee = 5,
        LeftAnkle = 6,
    };
    enum TriggerOutputMode
    {
        TriggerLongLOW = 0,
        TriggerPulse = 1,
        TriggerLongHigh = 2,
    };

    enum ExternalCANMode
    {
        CANTransmissionStopped = 0,
        CANTransmissionLowResolution = 1,
        CANTransmissionCaptureMode = 2,
        CANTransmissionHighResolution = 3,

    };
    enum MatrixIdentifiers
    {
        M_1_8_R = 1,
        M_2_16_R = 2,
        M_3_16_W = 3,
        M_4_16_R = 4,
        M_5_16_W = 5,
    };
} // namespace h3_definition