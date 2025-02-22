// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PoseL4CMD.h"
#include "subsystems/CoralSubsystem.h"

#include <iostream>

PoseL4CMD::PoseL4CMD() {
  // Use addRequirements() here to declare subsystem dependencies.
   //PoseL4CMD(CoralSubsystem& subsystem) : m_subsystem(subsystem) {
    // Add the subsystem as a requirement
    //AddRequirements(m_coralsubsystem); 
  //}

}

// Called when the command is initially scheduled.
void PoseL4CMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PoseL4CMD::Execute() {
  std::cout << "WOW!" << std::endl;
  //void SetEverything(double setElevator/*static number*/);

}

// Called once the command ends or is interrupted.
void PoseL4CMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PoseL4CMD::IsFinished() {
  return false;
}
