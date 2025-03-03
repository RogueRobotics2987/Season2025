// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/AlgaeSubsystem.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>


//algae intake arm movement

void AlgaeSubsystem::setIntakeMotors(double intakeSpeed){ //algae intake arm intake/out
    _intake.Set(intakeSpeed);
}


//algae removal arm movement