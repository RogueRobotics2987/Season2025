// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PoseL1CMD.h"

PoseL1CMD::PoseL1CMD() {}
PoseL1CMD::PoseL1CMD(CoralSubsystem &CoralSubsystem) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_coralSubsystem = &CoralSubsystem;
  AddRequirements(&CoralSubsystem);
}

// Called when the command is initially scheduled.
void PoseL1CMD::Initialize() 
{
  //set elevator to pose L1
  m_coralSubsystem->SetElevator(0);
}

// Called repeatedly when this Command is scheduled to run
void PoseL1CMD::Execute() 
{
  if(time >= 60)
  {
    timeIsUp = true;
  }
  else
  {
    time++;
  }
}

// Called once the command ends or is interrupted.
void PoseL1CMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PoseL1CMD::IsFinished() 
{
  return timeIsUp;
}
