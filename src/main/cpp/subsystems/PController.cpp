// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PController.h"

PController::PController(double in_kP) {
  // Use addRequirements() here to declare subsystem dependencies.
  kP = in_kP;
}

// Called when the command is initially scheduled.
void PController::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PController::Execute() {}

// Called once the command ends or is interrupted.
void PController::End(bool interrupted) {}

double PController::Calculate(double current, double setposition) {
  return (setposition - current) * kP;
}
// Returns true when the command should end.
bool PController::IsFinished() {
  return false;
}
