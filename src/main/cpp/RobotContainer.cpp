// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/PoseL4CMD.h"
#include "commands/RightSideApriltagReefLineup.h"
#include "subsystems/CoralSubsystem.h"
#include "subsystems/ClimberSubsystem.h" 
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <memory> 

using namespace pathplanner;

RobotContainer::RobotContainer()
{
    NamedCommands::registerCommand("PlaceCMD", std::move(PlaceCMD(m_coralSubsystem).ToPtr())); //NEEDS TO BE ABOVE CHOOSER
    NamedCommands::registerCommand("IntakeCMD", std::move(IntakeCMD(m_coralSubsystem).ToPtr()));
    NamedCommands::registerCommand("PoseL1CMD", std::move(PoseL1CMD(m_coralSubsystem).ToPtr()));
    NamedCommands::registerCommand("PoseL4CMD", std::move(PoseL4CMD(m_coralSubsystem).ToPtr()));
    NamedCommands::registerCommand("RightLineUp", std::move(RightSideApriltagReefLineup(drivetrain, m_lightSubsystem, rightBranchSetPointX, rightBranchSetPointY, rightBranchSetPointYaw, true).ToPtr()));
    NamedCommands::registerCommand("LeftLineUp", std::move(RightSideApriltagReefLineup(drivetrain, m_lightSubsystem, leftBranchSetPointX, leftBranchSetPointY, leftBranchSetPointYaw, false).ToPtr()));

    

    // Initialize all of your commands and subsystems here
    m_chooser = pathplanner::AutoBuilder::buildAutoChooser("tests"); //change name
    frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

    //public:
    // Configure the button bindings
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() // more needs to be added somewhere in here *look at GIT for what
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
         drivetrain.ApplyRequest([this]() -> auto &&
                                {
                                    units::volt_t value{DriveStick.GetRightTriggerAxis() + 0.25 > 1 ? 1 : DriveStick.GetRightTriggerAxis() + 0.25};
                                    units::volt_t outputMult = filter.Calculate(value);


                                    return drive.WithVelocityX(-DriveStick.GetLeftY() * MaxSpeed * outputMult.value())      // Drive forward with positive Y (forward)
                                        .WithVelocityY(-DriveStick.GetLeftX() * MaxSpeed * outputMult.value())              // Drive left with positive X (left)
                                        .WithRotationalRate(-DriveStick.GetRightX() * MaxAngularRate * outputMult.value()); // Drive counterclockwise with negative X (left)
                                }));

    DriveStick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    DriveStick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-DriveStick.GetLeftY(), -DriveStick.GetLeftX()});
    }));
    // DriveStick.POVUp().WhileTrue(m_coralSubsystem.SetArmAndElevator(L1Angle, L1Height));
    // DriveStick.Back().WhileTrue(frc2::InstantCommand([this]() -> void {
    //      m_coralSubsystem.ResetState();
    //      }).ToPtr());

    AuxStick.POVUp().WhileTrue(frc2::InstantCommand([this]() -> void { // L1 Button
                                   m_coralSubsystem.SetElevator(L1Height);
                               })
                                   .ToPtr());

    AuxStick.POVRight().WhileTrue(frc2::InstantCommand([this]() -> void { // L2 Button
                                      m_coralSubsystem.SetElevator(L2Height + GravityoffsetIn); // 9
                                  })
                                      .ToPtr());

    AuxStick.POVDown().WhileTrue(frc2::InstantCommand([this]() -> void { // L3 Button
                                     m_coralSubsystem.SetElevator(L3Height + GravityoffsetIn); // 25
                                 })
                                     .ToPtr());

    AuxStick.POVLeft().WhileTrue(frc2::InstantCommand([this]() -> void { // L4 Button
                                     m_coralSubsystem.SetElevator(L4Height + GravityoffsetIn); // 50.5
                                 })
                                     .ToPtr());

    AuxStick.RightTrigger().WhileTrue(frc2::RunCommand([this]() -> void { // Manual Elevator up
                                          m_coralSubsystem.ManualElevator(manualElevatorSpeedUp);
                                      })
                                          .ToPtr());

    AuxStick.LeftTrigger().WhileTrue(frc2::RunCommand([this]() -> void { // Manual Elevator down
                                         m_coralSubsystem.ManualElevator(manualElevatorSpeedDown);
                                     })
                                         .ToPtr());

    AuxStick.A().WhileTrue(frc2::InstantCommand([this]() -> void { // Intake Button and Place on
                               m_coralSubsystem.SetIntakeMotors(0.6);
                               m_lightSubsystem.GreenBlink();
                           })
                               .ToPtr());

    AuxStick.A().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Intake Button off
                                   m_coralSubsystem.SetIntakeMotors(0);
                                   m_lightSubsystem.Idle();
                               })
                                   .ToPtr());

    // AuxStick.B().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Eject Button on
    //                               m_coralSubsystem.SetIntakeMotors(-0.5);
    //                           })
    //                               .ToPtr());

    // AuxStick.B().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Eject Button off
    //                                m_coralSubsystem.SetIntakeMotors(0);
    //                            })
    //                                .ToPtr());

    DriveStick.X().ToggleOnTrue(frc2::InstantCommand([this]() -> void
                                                   { m_climberSubsystem.SetClimber(ClimberSpeed); })
                                    .ToPtr());

    DriveStick.X().ToggleOnFalse(frc2::InstantCommand([this]() -> void
                                                   { m_climberSubsystem.SetClimber(0); })
                                    .ToPtr());

    // DriveStick.Y().ToggleOnTrue(frc2::InstantCommand([this]() -> void
    //                                                { m_climberSubsystem.SetClimber(-20); })
    //                                 .ToPtr());

    // DriveStick.Y().ToggleOnFalse(frc2::InstantCommand([this]() -> void
    //                                                { m_climberSubsystem.SetClimber(0); })
    //                                 .ToPtr());                    

    AuxStick.RightBumper().ToggleOnTrue(frc2::InstantCommand([this]() -> void
                                                   { m_climberSubsystem.SetClimber(ClimberSpeed); })
                                    .ToPtr());

    AuxStick.RightBumper().ToggleOnFalse(frc2::InstantCommand([this]() -> void
                                                   { m_climberSubsystem.SetClimber(0); })
                                    .ToPtr());                                    

    AuxStick.Y().OnTrue(frc2::InstantCommand([this]() -> void
                                                   { m_coralSubsystem.SetAlgyArm(0.14); })
                                  .ToPtr());

    AuxStick.X().OnTrue(frc2::InstantCommand([this]() -> void
                                                   { m_coralSubsystem.SetAlgyArm(0); })
                                  .ToPtr());

    AuxStick.LeftBumper().WhileTrue(frc2::RunCommand([this]() -> void
                                                   { m_coralSubsystem.SetAlgyArmManual(0.005); })
                                  .ToPtr());

    AuxStick.LeftBumper().WhileFalse(frc2::RunCommand([this]() -> void
                                                   { m_coralSubsystem.SetAlgyArmManual(0); })
                                  .ToPtr());

    // AuxStick.RightBumper().ToggleOnTrue(frc2::InstantCommand([this]() -> void
    //                                                { m_coralSubsystem.SetAlgyArmManual(0.2); })
    //                                 .ToPtr());

    // AuxStick.RightBumper().ToggleOnFalse(frc2::InstantCommand([this]() -> void
    //                                                { m_coralSubsystem.SetAlgyArmManual(0); })
    //                                 .ToPtr());        

    DriveStick.Start().OnTrue(frc2::InstantCommand([this]() -> void
                                                        { m_coralSubsystem.SetFunnelPin(0.1);})
                                  .ToPtr());
                                  
    DriveStick.Start().OnFalse(frc2::InstantCommand([this]() -> void
                                                        { m_coralSubsystem.SetFunnelPin(0);})
                                  .ToPtr());

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (DriveStick.Back() && DriveStick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (DriveStick.Back() && DriveStick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (DriveStick.Start() && DriveStick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (DriveStick.Start() && DriveStick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    DriveStick.Back().WhileTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));
    //TODO: look at last years code and find out why its not being scheduled
    DriveStick.LeftBumper().WhileTrue(RightSideApriltagReefLineup(drivetrain, m_lightSubsystem, leftBranchSetPointX, leftBranchSetPointY, leftBranchSetPointYaw, true).ToPtr());
    DriveStick.RightBumper().WhileTrue(RightSideApriltagReefLineup(drivetrain, m_lightSubsystem, rightBranchSetPointX, rightBranchSetPointY, rightBranchSetPointYaw, false).ToPtr());

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
    // drivetrain.GetState().Pose; // TODO: Removed during auto line up merge
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    // return PathPlannerAuto("TestAuto").ToPtr();
    return m_chooser.GetSelected(); //*m_chooser compiles when this is not being returned. // TODO: Removed during auto line up merge
}