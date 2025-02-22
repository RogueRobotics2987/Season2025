// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/CoralSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
    m_chooser = pathplanner::AutoBuilder::buildAutoChooser("tests"); //change name
    frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);



    //public:
    NamedCommands::registerCommand("PlaceL4CMD", std::move(PlaceL4CMD().ToPtr()));
    NamedCommands::registerCommand("PlaceCMD", std::move(PlaceCMD().ToPtr()));

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
          units::volt_t value{(1 - 0.25) * DriveStick.GetRightTriggerAxis() + 0.25};
          units::volt_t outputMult = filter.Calculate(value);

            return drive.WithVelocityX(DriveStick.GetLeftY() * MaxSpeed * outputMult.value()) // Drive forward with positive Y (forward)
                .WithVelocityY(DriveStick.GetLeftX() * MaxSpeed * outputMult.value()) // Drive left with positive X (left)
                .WithRotationalRate(-DriveStick.GetRightX() * MaxAngularRate * outputMult.value()); // Drive counterclockwise with negative X (left)
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
    
    AuxStick.POVUp().WhileTrue(frc2::InstantCommand([this]() -> void { // L1 Button
          m_coralSubsystem.SetEverything(7.55);
         }).ToPtr());
         
    AuxStick.POVRight().WhileTrue(frc2::InstantCommand([this]() -> void { // L2 Button
        m_coralSubsystem.SetEverything(10.86);
         }).ToPtr());

    AuxStick.POVDown().WhileTrue(frc2::InstantCommand([this]() -> void { // L3 Button
         m_coralSubsystem.SetEverything(20.5);
         }).ToPtr());

    AuxStick.POVLeft().WhileTrue(frc2::InstantCommand([this]() -> void { // L4 Button
         m_coralSubsystem.SetEverything(21.16);
         }).ToPtr());

//     AuxStick.LeftTrigger().WhileTrue(frc2::InstantCommand([this]() -> void {
//          m_coralSubsystem.IncrementOffsets(0, 0.001, 0);
//          }).ToPtr());

//     AuxStick.RightTrigger().WhileTrue(frc2::InstantCommand([this]() -> void {
//         m_coralSubsystem.IncrementOffsets(0, -0.001, 0);
//          }).ToPtr());

    AuxStick.A().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Intake Button
        m_coralSubsystem.SetIntakeMotors(0.2);
         }).ToPtr());

    AuxStick.B().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Eject Button
        m_coralSubsystem.SetIntakeMotors(-0.1);
         }).ToPtr());

    AuxStick.A().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Intake Off Button
        m_coralSubsystem.SetIntakeMotors(0);
         }).ToPtr());

    AuxStick.B().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Eject Off Button
        m_coralSubsystem.SetIntakeMotors(0);
         }).ToPtr());

    AuxStick.LeftBumper().ToggleOnTrue(frc2::InstantCommand([this]() -> void {
         m_coralSubsystem.SetEverything(11);
         m_coralSubsystem.SetIntakeMotors(0);
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
    drivetrain.GetState().Pose;
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
   return m_chooser.GetSelected(); //*m_chooser compiles when this is not being returned

    //auto path = PathPlannerPath::fromPathFile("TestMoveOutPath");

    //return PathPlannerAuto("TestAuto").ToPtr();

}