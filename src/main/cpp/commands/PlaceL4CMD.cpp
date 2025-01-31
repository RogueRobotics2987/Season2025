// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceL4CMD.h"
#include "subsystems/CoralSubsystem.h"

#include <iostream>

PlaceL4CMD::PlaceL4CMD() {
  // Use addRequirements() here to declare subsystem dependencies.
   //PlaceL4CMD(CoralSubsystem& subsystem) : m_subsystem(subsystem) {
    // Add the subsystem as a requirement
    //AddRequirements(m_coralsubsystem); 
  //}

}

// Called when the command is initially scheduled.
void PlaceL4CMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PlaceL4CMD::Execute() {
  //void Place_armAndElevatorL4(double setArmAngle, double setElevatorHeight, bool setCoralPlace);
  std::cout << "WOW!" << std::endl;

}

// Called once the command ends or is interrupted.
void PlaceL4CMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PlaceL4CMD::IsFinished() {
  return false;
}
