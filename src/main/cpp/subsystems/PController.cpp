// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PController.h"

// Constructor
PController::PController(double in_kP){
  // Use addRequirements() here to declare subsystem dependencies.
 kP = in_kP;
}

// Called when the command is initially scheduled.
// Name : Initialize
void PController::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PController::Execute() {}

// Called once the command ends or is interrupted.
void PController::End(bool interrupted) {}

double PController::Calculate(double current, double setpoint){
 double err = current - setpoint;
 double output = err * kP;
 return output;
 }
// Returns true when the command should end.
// Return type: Bool
bool PController::IsFinished() {
  return false;
}
