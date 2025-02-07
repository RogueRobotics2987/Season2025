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
    // 0 arm angle is when the arm is pointing down and 0 height is when the elevator is all the way down
    // all elevator measurement are in inches and will need changed to meters
    // all angle measurements are in degrees
    inline constexpr rev::spark::SparkMax::MotorType NeoMotorType = rev::spark::SparkMax::MotorType::kBrushless;
    inline constexpr int CANIdLeaderElevatorFirstStage = 48;
    inline constexpr int CANIdFollowerElevatorFirstStage = 1;
    inline constexpr int CANIdElevatorSecondStage = 2;
    inline constexpr int CANIdGrabberArm = 3;
    inline constexpr int CANIdLeftIntake = 4;
    inline constexpr int CANIdRightIntake = 5;
    inline constexpr double restingArmAngle = 0; 
    inline constexpr double restingElevatorHeight = 22.5; 
    inline constexpr double elevatorZeroReverseSpeed = -0.25;
    inline constexpr double firstStageMaxElevatorHeight = 24; 
    inline constexpr double firstStageMinElevatorHeight = 8; 
    inline constexpr double secondStageMaxElevatorHeight = 25.5; 
    inline constexpr double maxElevatorHeight = 51; 
    inline constexpr double maxArmAngle = 175; 
    inline constexpr double minArmAngle = 0; 
    inline constexpr double safetyArmAngle = 45;
    inline constexpr double safetyElevatorHeight = 22.5;
    inline constexpr double intakeHeight = 17.5;
    inline constexpr double intakeArmAngle = 0;
    inline constexpr double intakeSpeed = 0.1; // this number is lower for testing
    inline constexpr double placingArmAngle = 125;
    inline constexpr double armLowered = 30;
}