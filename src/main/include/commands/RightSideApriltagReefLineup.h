// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"
#include "RobotContainer.h"
#include "Telemetry.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RightSideApriltagReefLineup
    : public frc2::CommandHelper<frc2::Command, RightSideApriltagReefLineup> {
  
  public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  RightSideApriltagReefLineup();
  RightSideApriltagReefLineup(Telemetry* &drivePose, subsystems::CommandSwerveDrivetrain* &pose, RobotContainer* &driveTrain); // needs xbox perm? //dont think we need drivepose

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  float Deadzone(float x);

  frc::XboxController* m_driverController = nullptr; // make into smart pointer?
  frc::XboxController* m_auxController = nullptr; // make into smart pointer?

  units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);

  std::vector<int> _reefTags{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}; //reef Apriltags 6-11 & 17-22, red: 6-11 blue: 17-22

  int state = 0;
  int time = 0;
  int direction = 1;

  private: //put variables where they should go

  subsystems::CommandSwerveDrivetrain* m_drive = nullptr;
  Telemetry* m_drivePose = nullptr;
  RobotContainer* _driveTrain = nullptr;

  double currentHeading = 0;
  double lastHeading = 0;
  double speedX = 0;
  double speedY = 0;
  double rot = 0; //some of these are in private some arent

  bool NoJoystickInput = false;
  bool hasSeen = false;
  bool finished = false;
};
