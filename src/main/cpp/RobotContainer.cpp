// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

#include "TeleopCurve.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    /*double withVelocityX = TeleopCurve::apply(-joystick.GetLeftY());
    double withVelocityY = TeleopCurve::apply(joystick.GetLeftX());
    double withRotationalRate = TeleopCurve::apply(-joystick.GetRightX());*/
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(TeleopCurve::apply(-joystick.GetLeftY()) * MaxSpeed) // Drive forward with positive Y (forward)   return drive.WithVelocityX(TeleopCurve::apply(joystick.GetLeftY()) * MaxSpeed)
                .WithVelocityY(TeleopCurve::apply(joystick.GetLeftX()) * MaxSpeed) // Drive left with positive X (left)
                .WithRotationalRate(TeleopCurve::apply(-joystick.GetRightX()) * MaxAngularRate); // Drive counterclockwise with negative X (left)
            /*std::string WithVelocityX = drive.WithVelocityX(TeleopCurve::apply(joystick.GetLeftY()) * MaxSpeed)
            frc::SmartDashboard::PutNumber("With velocity X", WithVelocityX);*/
    }));

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    frc::SmartDashboard::PutNumber("WithVelocityX", TeleopCurve::apply(-joystick.GetLeftY()));
    frc::SmartDashboard::PutNumber("WithVelocityY", TeleopCurve::apply(joystick.GetLeftX()));
    frc::SmartDashboard::PutNumber("WithRotationalRate",TeleopCurve::apply(-joystick.GetRightX()));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}
