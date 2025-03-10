// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void SetClimber(double ClimberSpeed);

  SparkClosedLoopController _climberclosedLoopController = _climber.GetClosedLoopController();
  SparkRelativeEncoder _climberencoder = _climber.GetEncoder();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 SparkMax _climber{ClimberSubsystemConstants::CANIdClimber, SparkMax::MotorType::kBrushless};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
