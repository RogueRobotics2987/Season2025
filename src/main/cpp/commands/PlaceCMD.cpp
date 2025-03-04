// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceCMD.h"
#include <iostream>

PlaceCMD::PlaceCMD() {}
PlaceCMD::PlaceCMD(CoralSubsystem &CoralSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_coralSubsystem = &CoralSubsystem;
  AddRequirements(&CoralSubsystem);
}

// Called when the command is initially scheduled.
void PlaceCMD::Initialize() 
{
  timeIsUp = false;
  time = 0;
  
  m_coralSubsystem->SetIntakeMotors(0.3);
}

// Called repeatedly when this Command is scheduled to run
void PlaceCMD::Execute() 
{
  //std::cout << "WOAH" << std::endl;
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
void PlaceCMD::End(bool interrupted) 
{
  m_coralSubsystem->SetIntakeMotors(0);
}

// Returns true when the command should end.
bool PlaceCMD::IsFinished() 
{
  return timeIsUp;
}
