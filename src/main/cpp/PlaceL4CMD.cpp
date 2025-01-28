// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PlaceL4CMD.h"
#include "subsystems/CoralSubsystem.h"

CommanPlaceL4::CommanPlaceL4() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CommanPlaceL4::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CommanPlaceL4::Execute() {
  void Place_armAndElevatorL4(double setArmAngle, double setElevatorHeight, bool setCoralPlace);

}

// Called once the command ends or is interrupted.
void CommanPlaceL4::End(bool interrupted) {}

// Returns true when the command should end.
bool CommanPlaceL4::IsFinished() {
  return false;
}
