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
    inline constexpr int CANIdLeftElevator = 48;
    inline constexpr int CANIdRightElevator = 2;
    inline constexpr int CANIdGrabberArm = 3;
    inline constexpr int CANIdLeftIntake = 4;
    inline constexpr int CANIdRightIntake = 5;
    inline constexpr rev::spark::SparkMax::MotorType NeoMotorType = rev::spark::SparkMax::MotorType::kBrushless;
    inline constexpr double restingArmAngle = -90;
    inline constexpr double restingElevatorHeight = 0;
}