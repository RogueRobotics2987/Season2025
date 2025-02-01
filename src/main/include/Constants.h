// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/SparkMax.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants


namespace CoralSubsystemConstants {
    inline constexpr int CANIdLeaderElevatorFirstStage = 48;
    inline constexpr int CANIdFollowerElevatorFirstStage = 1;
    inline constexpr int CANIdElevatorSecondStage = 2;
    inline constexpr int CANIdGrabberArm = 3;
    inline constexpr int CANIdLeftIntake = 4;
    inline constexpr int CANIdRightIntake = 5;
    inline constexpr rev::spark::SparkMax::MotorType NeoMotorType = rev::spark::SparkMax::MotorType::kBrushless;
    inline constexpr double restingArmAngle = 0; // this number is in degrees
    inline constexpr double restingElevatorHeight = 22.5; // this number is in inches
    inline constexpr double elevatorZeroReverseSpeed = -0.25;
    inline constexpr double firstStageMaxElevatorHeight = 24; // this number is in inches
    inline constexpr double firstStageMinElevatorHeight = 8; // this number is in inches
    inline constexpr double secondStageMaxElevatorHeight = 32; // this number is in inches
    inline constexpr double maxElevatorHeight = 56; // this number is in inches
    inline constexpr double maxArmAngle = 175; // this number is in degrees
    inline constexpr double minArmAngle = 0; // this number is in degrees
    inline constexpr double safetyArmAngle = 45;
    inline constexpr double safetyElevatorHeight = 22.5;
}