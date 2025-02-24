// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PoseL3CMD.h"

PoseL3CMD::PoseL3CMD() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PoseL3CMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PoseL3CMD::Execute() 
{
  //set elevator to pose L3
  void SetEverything(double setElevator /*static number*/);  
}

// Called once the command ends or is interrupted.
void PoseL3CMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PoseL3CMD::IsFinished() {
  return false;
}
