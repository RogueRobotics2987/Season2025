// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/CoralSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "Deadzone.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

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
          units::volt_t value{(1 - 0.25) * DriveStick.GetRightTriggerAxis() + 0.25};
          units::volt_t outputMult = filter.Calculate(value);
        
        frc::SmartDashboard::PutNumber("Input Number", -DriveStick.GetLeftY()); // debugging values
        frc::SmartDashboard::PutNumber("Output Number", -DriveStick.GetLeftY() * outputMult.value()); // debugging values

            return drive.WithVelocityX(Deadzone::applyDeadzone(-DriveStick.GetLeftY()) * MaxSpeed * outputMult.value()) // Drive forward with positive Y (forward)
                .WithVelocityY(Deadzone::applyDeadzone(-DriveStick.GetLeftX()) * MaxSpeed * outputMult.value()) // Drive left with positive X (left)
                .WithRotationalRate(Deadzone::applyDeadzone(-DriveStick.GetRightX()) * MaxAngularRate * outputMult.value()); // Drive counterclockwise with negative X (left)
        })
    );

    // DriveStick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    // DriveStick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    //     return point.WithModuleDirection(frc::Rotation2d{-DriveStick.GetLeftY(), -DriveStick.GetLeftX()});
    // }));
    // DriveStick.POVUp().WhileTrue(m_coralSubsystem.SetArmAndElevator(L1Angle, L1Height));
    // DriveStick.Back().WhileTrue(frc2::InstantCommand([this]() -> void {
    //      m_coralSubsystem.ResetState();
    //      }).ToPtr());
    
    AuxStick.X().WhileTrue(frc2::InstantCommand([this]() -> void { // L1 Button
          m_coralSubsystem.SetElevator(0);
         }).ToPtr());
         
    AuxStick.A().WhileTrue(frc2::InstantCommand([this]() -> void { // L2 Button
        m_coralSubsystem.SetElevator(10.86);
         }).ToPtr());

    AuxStick.B().WhileTrue(frc2::InstantCommand([this]() -> void { // L3 Button
         m_coralSubsystem.SetElevator(20.5);
         }).ToPtr());

    AuxStick.Y().WhileTrue(frc2::InstantCommand([this]() -> void { // L4 Button
         m_coralSubsystem.SetElevator(21.16);
         }).ToPtr());

    AuxStick.RightTrigger().WhileTrue(frc2::InstantCommand([this]() -> void { // manual elevator up
         m_coralSubsystem.IncrementOffsets(0.01);
         }).ToPtr());

    AuxStick.LeftTrigger().WhileTrue(frc2::InstantCommand([this]() -> void { // manual elevator down
        m_coralSubsystem.IncrementOffsets(-0.01);
         }).ToPtr());

    AuxStick.POVRight().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Intake Button and eject
        m_coralSubsystem.SetIntakeMotors(0.2);
         }).ToPtr());

    AuxStick.POVRight().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Intake Off Button
        m_coralSubsystem.SetIntakeMotors(0);
         }).ToPtr());

    AuxStick.POVLeft().WhileTrue(frc2::InstantCommand([this]() -> void { // Coral Place
        m_coralSubsystem.SetIntakeMotors(0.2);
         }).ToPtr());

    AuxStick.POVDown().WhileTrue(frc2::InstantCommand([this]() -> void { // intake preset
         m_coralSubsystem.SetElevator(0);
         m_coralSubsystem.SetIntakeMotors(0.2);
         }).ToPtr());

    DriveStick.Y().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Climber up
         m_climberSubsystem.SetClimberSpeed(0.25);
         }).ToPtr());

    DriveStick.Y().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Climber up
         m_climberSubsystem.SetClimberSpeed(0);
         }).ToPtr());

    DriveStick.X().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Climber down
         m_climberSubsystem.SetClimberSpeed(-0.25);
         }).ToPtr());

    DriveStick.X().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Climber down
         m_climberSubsystem.SetClimberSpeed(0);
         }).ToPtr());

    DriveStick.LeftBumper().WhileTrue(frc2::InstantCommand([this]() -> void { // Move to Reef Via Apriltag (left side)
         // to do
         }).ToPtr());

    DriveStick.RightBumper().WhileTrue(frc2::InstantCommand([this]() -> void { // Move to Reef Via Apriltag (right side)
         // to do
         }).ToPtr());

    DriveStick.Back().WhileTrue(frc2::InstantCommand([this]() -> void { // Reset heading
        drivetrain.ResetRotation(frc::Rotation2d(units::degree_t(0)));
         }).ToPtr());
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (DriveStick.Back() && DriveStick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (DriveStick.Back() && DriveStick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (DriveStick.Start() && DriveStick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (DriveStick.Start() && DriveStick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    DriveStick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}