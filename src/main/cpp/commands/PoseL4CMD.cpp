// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PoseL4CMD.h"
#include <iostream>

PoseL4CMD::PoseL4CMD() {}
PoseL4CMD::PoseL4CMD(CoralSubsystem &CoralSubsystem) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_coralSubsystem = &CoralSubsystem;
  AddRequirements(&CoralSubsystem);
}

// Called when the command is initially scheduled.
void PoseL4CMD::Initialize() 
{
  m_coralSubsystem->SetElevator(L4Height + GravityoffsetIn);
}

// Called repeatedly when this Command is scheduled to run
void PoseL4CMD::Execute() {}

// Called once the command ends or is interrupted.
void PoseL4CMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PoseL4CMD::IsFinished() 
{
  double error = std::abs(m_coralSubsystem->_elevatorLeader.GetEncoder().GetPosition() - (L4Height + GravityoffsetIn));
 
  if(error < 6)
  {
    return true;
  }else
  {
    return false;
  }
}
