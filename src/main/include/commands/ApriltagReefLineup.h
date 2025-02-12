// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CommandSwerveDrivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ApriltagReefLineup
    : public frc2::CommandHelper<frc2::Command, ApriltagReefLineup> {
  
  public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  ApriltagReefLineup();
  ApriltagReefLineup(Telemetry &drivePose, RobotContainer &drivetrain); // needs xbox perm?

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  float Deadzone(float x);

  frc::XboxController* m_driverController = nullptr; // make into smart pointer?
  frc::XboxController* m_auxController = nullptr; // make into smart pointer?

  units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);

  

  int state = 0;
  int time = 0;
  int direction = 1;

  private: //put variables where they should go

  RobotContainer* m_drive = nullptr;
  Telemetry* m_drivePose = nullptr;

  double currentHeading = 0;
  double lastHeading = 0;
  double speedX = 0;
  double speedY = 0;
  double rot = 0; //some of these are in private some arent

  bool NoJoystickInput = false;
  bool hasSeen = false;
  bool finished = false;
};
