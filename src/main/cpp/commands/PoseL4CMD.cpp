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
  m_coralSubsystem->SetElevator(50.5 + GravityoffsetIn);
  //m_coralSubsystem->SetIntakeMotors(0.3);
}

// Called repeatedly when this Command is scheduled to run
void PoseL4CMD::Execute() 
{
  // if(time >= 60)
  // {
  //   timeIsUp = true;
  // }
  // else
  // {
  //   time++;
  // }
}

// Called once the command ends or is interrupted.
void PoseL4CMD::End(bool interrupted) 
{
  //m_coralSubsystem->SetElevator(0);
}

// Returns true when the command should end.
bool PoseL4CMD::IsFinished() 
{
  double error = std::abs(m_coralSubsystem->_elevatorLeader.GetEncoder().GetPosition() - (50.5 + GravityoffsetIn));
  //std::cout << error << std::endl;
  if(error < 6)
  {
    //std::cout << "it is ending" << std::endl;
    return true;
  }else
  {
    return false;
  }
  // return timeIsUp;
}
