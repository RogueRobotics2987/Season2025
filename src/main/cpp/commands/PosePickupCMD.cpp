// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PosePickupCMD.h"

PosePickupCMD::PosePickupCMD() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PosePickupCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PosePickupCMD::Execute() 
{
  //set elevator all the way to pick up the coral from the funnel
  void SetEverything(double setElevator /*static number*/);  
}

// Called once the command ends or is interrupted.
void PosePickupCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PosePickupCMD::IsFinished() {
  return false;
}
