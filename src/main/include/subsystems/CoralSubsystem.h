// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

enum PossibleStates {
  EMPTY,
  CORAL_IN_FUNNEL,
  CORAL_IN_TROUGH,
  ALLOW_CORAL_MOVE,
  CORAL_PLACE
 };

class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  enum PossibleStates _state = EMPTY;
    SparkMax _elavatorLeft{CoralSubsystemConstants::CANIdLeftElevator, SparkMax::MotorType::kBrushless};
    SparkMax _elavatorRight{CoralSubsystemConstants::CANIdRightElevator, SparkMax::MotorType::kBrushless};
    SparkMax _grabberArm{CoralSubsystemConstants::CANIdGrabberArm, SparkMax::MotorType::kBrushless};
    SparkMax _intakeLeft{CoralSubsystemConstants::CANIdLeftIntake, SparkMax::MotorType::kBrushless};
    SparkMax _intakeRight{CoralSubsystemConstants::CANIdRightIntake, SparkMax::MotorType::kBrushless};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
