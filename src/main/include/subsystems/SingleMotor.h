// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
class SingleMotor : public frc2::SubsystemBase {
 public:
  SingleMotor(); // Constructor

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;  //periodic=function, void=return type

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

// I want a CANSparkMax motor controller object with CANid=1 and brushless, I 
// Want it to be called m_motor1

static const int CANid = 1;
rev::CANSparkMax m_motor1 {CANid, rev::CANSparkMax::MotorType::kBrushless};
};
