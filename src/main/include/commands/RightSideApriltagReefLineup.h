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
  RightSideApriltagReefLineup(subsystems::CommandSwerveDrivetrain &driveTrain, RobotContainer &robotContainer); // needs xbox perm? //dont think we need drivepose

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

  // add serve::requests::robotcentric drive object here
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
  units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

  swerve::requests::RobotCentric robotCentricDrive = swerve::requests::RobotCentric{}
  .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors

  subsystems::CommandSwerveDrivetrain _driveTrain;
  RobotContainer _robotContainer;
  
  double currentHeading = 0;
  double lastHeading = 0;
  double speedX = 0;
  double speedY = 0;
  double rot = 0; //some of these are in private some arent
  double kP_x = 0.1;
  double kP_yaw = 0.1;
  double errorX;
  double erroryaw;
  double errorY; //still need to calculate even though its not in PID
  double currentyaw;
  
  int currentx;

  bool NoJoystickInput = false;
  bool hasSeen = false;
  bool finished = false;
};
