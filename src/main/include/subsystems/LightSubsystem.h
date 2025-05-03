// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalOutput.h>
class LightSubsystem : public frc2::SubsystemBase {
 public:
  LightSubsystem();
  void Off();
  void Idle();
  void GreenBlink();
  void Green();
  void RedBlink();
  void Red();
  void BlueBlink();
  void Blue();
  frc::DigitalOutput _light1{0};
  frc::DigitalOutput _light2{1};
  frc::DigitalOutput _light3{2};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};