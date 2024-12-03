// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SingleMotor.h"

SingleMotor::SingleMotor() = default;

// This method will be called once per scheduler run

// Mkae the notor drive forwards at 0.25 (25%), use the  .Set( ... ) function to make the
// motor move
void SingleMotor::Periodic() 
{
    m_motor1.Set(0.25);
}
