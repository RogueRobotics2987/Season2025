// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "Constants.h"
#include "subsystems/CoralSubsystem.h"
#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include <frc2/command/RunCommand.h>
#include <frc/filter/SlewRateLimiter.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */

class RobotContainer
{

public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand(); // smart pointer because pathplanner LIB sendable chooser
  frc::SlewRateLimiter<units::volts> filter{4_V / 1_s};

private:
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
  units::radians_per_second_t MaxAngularRate = 0.75_tps;                 // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
                                             .WithDeadband(MaxSpeed * 0.03)
                                             .WithRotationalDeadband(MaxAngularRate * 0.03)                     // Add a 10% deadband
                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
  swerve::requests::SwerveDriveBrake brake{};
  swerve::requests::PointWheelsAt point{};

  /* Note: This must be constructed before the drivetrain, otherwise we need to
   *       define a destructor to un-register the telemetry from the drivetrain */
  Telemetry logger{MaxSpeed};

  frc2::CommandXboxController DriveStick{0};
  frc2::CommandXboxController AuxStick{1};

  subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

  double elevatorOffset = 0;

  frc::SendableChooser<frc2::Command *> m_chooser;

  // The robot's subsystems are defined here...
  CoralSubsystem m_coralSubsystem;

  void ConfigureBindings();
};
