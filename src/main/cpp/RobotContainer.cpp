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
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            frc::SmartDashboard::PutNumber("Input Number", -joystick.GetLeftY());
            if (drivetrainMode == 0) { // If default (fast) mode apply fast
                frc::SmartDashboard::PutNumber("Output Number", TeleopCurve::applyFast(-joystick.GetLeftY()));
                return drive.WithVelocityX(TeleopCurve::applyFast(-joystick.GetLeftY()) * MaxSpeed) // Drive forward with positive Y (forward)   return drive.WithVelocityX(TeleopCurve::apply(joystick.GetLeftY()) * MaxSpeed)
                    .WithVelocityY(TeleopCurve::applyFast(joystick.GetLeftX()) * -MaxSpeed) // Drive left with positive X (left)
                    .WithRotationalRate(TeleopCurve::applyFast(-joystick.GetRightX()) * MaxAngularRate); // Drive counterclockwise with negative X (left)    
            } else { // otherwise apply fine
                frc::SmartDashboard::PutNumber("Output Number", TeleopCurve::applyFine(-joystick.GetLeftY()));
                return drive.WithVelocityX(TeleopCurve::applyFine(-joystick.GetLeftY()) * MaxSpeed) // Drive forward with positive Y (forward)   return drive.WithVelocityX(TeleopCurve::apply(joystick.GetLeftY()) * MaxSpeed)
                    .WithVelocityY(TeleopCurve::applyFine(joystick.GetLeftX()) * -MaxSpeed) // Drive left with positive X (left)
                    .WithRotationalRate(TeleopCurve::applyFine(-joystick.GetRightX()) * MaxAngularRate); // Drive counterclockwise with negative X (left)          
            }}));

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

    joystick.Start().OnTrue(drivetrain.RunOnce([this] { // Detects Start button (small with three lines) and changes variable for mode
         frc::SmartDashboard::PutNumber("Drive Mode", drivetrainMode);
        if (drivetrainMode == 1) {
            drivetrainMode = 0;
        } else {
            drivetrainMode = 1;
            frc::SmartDashboard::PutNumber("Drivetrain Mode", drivetrainMode);
        }
    }));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}
