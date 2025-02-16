// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/CoralSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/PlaceL4CMD.h"

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <memory> 

using namespace pathplanner;
     
RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here
    //m_chooser = pathplanner::AutoBuilder::buildAutoChooser("tests");
    //frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

    //public:
    NamedCommands::registerCommand("PlaceL4CMD", std::move(PlaceL4CMD().ToPtr()));
    // Configure the button bindings
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() // more needs to be added somewhere in here *look at GIT for what
{
    
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with positive Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with positive X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    // joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    //     return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    // }));
    // joystick.POVUp().WhileTrue(m_coralSubsystem.SetArmAndElevator(L1Angle, L1Height));
    // joystick.Back().WhileTrue(frc2::InstantCommand([this]() -> void {
    //      m_coralSubsystem.ResetState();
    //      }).ToPtr());
    joystick.POVUp().WhileTrue(frc2::InstantCommand([this]() -> void { // L1 Button
          m_coralSubsystem.SetEverything(0.3, 7.55, 0);
         }).ToPtr());
         
    joystick.POVRight().WhileTrue(frc2::InstantCommand([this]() -> void { // L2 Button
         m_coralSubsystem.SetEverything(0.35, 10.86, 0);
         }).ToPtr());

    joystick.POVDown().WhileTrue(frc2::InstantCommand([this]() -> void { // L3 Button
         m_coralSubsystem.SetEverything(0.35, 17.07, 1.167);
         }).ToPtr());

    joystick.POVLeft().WhileTrue(frc2::InstantCommand([this]() -> void { // L4 Button
         m_coralSubsystem.SetEverything(0.389, 21.16, 9.45);
         }).ToPtr());

    joystick.LeftTrigger().WhileTrue(frc2::InstantCommand([this]() -> void {
         m_coralSubsystem.IncrementOffsets(0, 0.001, 0);
         }).ToPtr());

    joystick.RightTrigger().WhileTrue(frc2::InstantCommand([this]() -> void {
        m_coralSubsystem.IncrementOffsets(0, 0.001, 0);
         }).ToPtr());

    joystick.A().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Intake Button
        m_coralSubsystem.SetIntakeMotors(0.2);
         }).ToPtr());

    joystick.B().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Eject Button
        m_coralSubsystem.SetIntakeMotors(-0.1);
         }).ToPtr());

    joystick.A().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Intake Off Button
        m_coralSubsystem.SetIntakeMotors(0);
         }).ToPtr());

    joystick.B().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Eject Off Button
        m_coralSubsystem.SetIntakeMotors(0);
         }).ToPtr());

    joystick.LeftBumper().ToggleOnTrue(frc2::InstantCommand([this]() -> void {
         m_coralSubsystem.SetEverything(0, 4.9, 0);
         m_coralSubsystem.SetIntakeMotors(0);
         }).ToPtr());

    joystick.RightBumper().ToggleOnTrue(frc2::InstantCommand([this]() -> void {
         m_coralSubsystem.SetEverything(0, 6, 0);
         m_coralSubsystem.SetIntakeMotors(0.2);
         }).ToPtr());

    
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
    drivetrain.GetState().Pose;
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    //return m_chooser.GetSelected(); //*m_chooser compiles when this is not being returned

    //auto path = PathPlannerPath::fromPathFile("TestMoveOutPath");

    //return frc2::cmd::Print{"No autonomous command configured"};

    return PathPlannerAuto("TestAuto").ToPtr();

    //return AutoBuilder::followPath(path);
}