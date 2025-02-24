// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceCMD.h"

PlaceCMD::PlaceCMD() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PlaceCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PlaceCMD::Execute() 
{
  void SetIntakeMotors(double intakeSpeed /*static number*/); //run the motors backwards to place on any branch after wait 
}

// Called once the command ends or is interrupted.
void PlaceCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PlaceCMD::IsFinished() {
  return false;
}
