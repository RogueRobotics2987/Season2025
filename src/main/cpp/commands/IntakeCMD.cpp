// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCMD.h"

IntakeCMD::IntakeCMD() {}
IntakeCMD::IntakeCMD(CoralSubsystem &CoralSubsystem)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_coralSubsystem = &CoralSubsystem;
  AddRequirements(&CoralSubsystem);
}

// Called when the command is initially scheduled.
void IntakeCMD::Initialize() 
{
  if(!m_coralSubsystem->coralLoaded){
    m_coralSubsystem->SetIntakeMotors(intakeSpeed); 
  }
}

// Called repeatedly when this Command is scheduled to run
void IntakeCMD::Execute() {}

// Called once the command ends or is interrupted.
void IntakeCMD::End(bool interrupted) 
{
  m_coralSubsystem->SetIntakeMotors(0);
}

// Returns true when the command should end.
bool IntakeCMD::IsFinished() 
{
  return m_coralSubsystem->coralLoaded;
}
