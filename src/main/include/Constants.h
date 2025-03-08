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

namespace ClimberSubsystemConstants {
    inline constexpr int CANIdClimber = 25;
}


namespace CoralSubsystemConstants {
    // 0 arm angle is when the arm is pointing down and 0 height is when the elevator is all the way down
    // all elevator measurement are in meters
    // all angle measurements are in degrees

    // TODO: All these's numbers will be changed
    inline constexpr rev::spark::SparkMax::MotorType NeoMotorType = rev::spark::SparkMax::MotorType::kBrushless;
    inline constexpr int CANIdLeaderElevator = 21;
    inline constexpr int CANIdFollowerElevator = 20;
    inline constexpr int CANIdTopIntake = 22;
    inline constexpr int CANIdAlgyArm = 27;
    // inline constexpr int CANIdClimber = 25;
    // inline constexpr int CANIdAlgaePlacerWheel = 26;
    // inline constexpr int CANIdAlgaePlacerArm = 27;

    inline constexpr double restingElevatorHeight = 0;
    inline constexpr double elevatorZeroReverseSpeed = -0.25;
    inline constexpr double minElevatorHeight = 0;
    inline constexpr double maxElevatorHeight = 1.3335; 
    inline constexpr double GravityoffsetIn = 3.8;

    inline constexpr double L1Height = 0.74;
    inline constexpr double L2Height = 1.07;
    inline constexpr double L3Height = 1.24;
    inline constexpr double L4Height = 1.3335;

    inline constexpr double safetyElevatorHeight = 0.57;
    inline constexpr double intakeHeight = 0.44;
    inline constexpr double intakeSpeed = 0.1;
    inline constexpr double intakeOff = 0;
    // inline constexpr double L1 = 0.74;
    // inline constexpr double L2 = 1.07;
    // inline constexpr double L3 = 1.24;
    // inline constexpr double L4 = 1.42;
}