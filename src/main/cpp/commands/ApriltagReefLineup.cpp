// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "commands/ApriltagReefLineup.h"

ApriltagReefLineup::ApriltagReefLineup() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ApriltagReefLineup::Initialize() {
  state = 0;
  time = 0;
  finished = false;
}

// Called repeatedly when this Command is scheduled to run
void ApriltagReefLineup::Execute() {
  frc::SmartDashboard::PutBoolean("AutoLineup", true); //bool or string??


}

// Called once the command ends or is interrupted.
void ApriltagReefLineup::End(bool interrupted) {}

// Returns true when the command should end.
bool ApriltagReefLineup::IsFinished() {
  return false;
}
