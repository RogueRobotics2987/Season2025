// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalOutput.h>
class LightSubsystem : public frc2::SubsystemBase {
 public:
  LightSubsystem();
  void LightsOff();
  void RBSwap();
  void LightsPink();
  void LightsCyan();
  void PinkBlink();
  void CyanBlink();
  frc::DigitalOutput _light1{3};
  frc::DigitalOutput _light2{4};
  frc::DigitalOutput _light3{5};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};