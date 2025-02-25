// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceCMD.h"

PlaceCMD::PlaceCMD() {}
PlaceCMD::PlaceCMD(CoralSubsystem &CoralSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_coralSubsystem = &CoralSubsystem;
  AddRequirements(&CoralSubsystem);
}

// Called when the command is initially scheduled.
void PlaceCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PlaceCMD::Execute() 
{
  m_coralSubsystem->SetIntakeMotors(0.2); //run the motors backwards to place on any branch after wait 
}

// Called once the command ends or is interrupted.
void PlaceCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PlaceCMD::IsFinished() {
  return false;
}
