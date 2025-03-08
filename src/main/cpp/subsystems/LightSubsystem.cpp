// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LightSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

LightSubsystem::LightSubsystem() = default;

void LightSubsystem::LightsOff() {
    _light1.Set(false);
    _light2.Set(false);
    _light3.Set(false);
  }
  void LightSubsystem::RBSwap() {
    _light1.Set(true);
    _light2.Set(false);
    _light3.Set(false);
  }
 void LightSubsystem::LightsPink() {
    _light1.Set(false);
    _light2.Set(true);
    _light3.Set(false);
  }
  void LightSubsystem::LightsCyan() {
    _light1.Set(false);
    _light2.Set(false);
    _light3.Set(true);
  }
  void LightSubsystem::PinkBlink() {
    _light1.Set(true);
    _light2.Set(true);
    _light3.Set(false);
  }
  void LightSubsystem::CyanBlink(){
    _light1.Set(true);
    _light2.Set(false);
    _light3.Set(true);
  }

// This method will be called once per scheduler run
void LightSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("_light1", _light1.Get());
    frc::SmartDashboard::PutBoolean("_light2", _light2.Get());
    frc::SmartDashboard::PutBoolean("_light3", _light3.Get());
}