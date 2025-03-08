// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/CoralSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/AlgaeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
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
        

            return drive.WithVelocityX(-DriveStick.GetLeftY() * MaxSpeed * outputMult.value()) // Drive forward with positive Y (forward)
                .WithVelocityY(-DriveStick.GetLeftX() * MaxSpeed * outputMult.value()) // Drive left with positive X (left)
                .WithRotationalRate(-DriveStick.GetRightX() * MaxAngularRate * outputMult.value()); // Drive counterclockwise with negative X (left)
        })
    );

    m_algaeSubsystem.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                double LeftY = AuxStick.GetLeftY();
                if (LeftY > 0.1) { 
                    m_algaeSubsystem.setAlgaeArm(LeftY);
                } else if (LeftY < -0.1) {
                    m_algaeSubsystem.setAlgaeArm(LeftY);
                }

                double RightY = AuxStick.GetRightY();
                if (RightY > 0.1) { 
                    m_algaeSubsystem.setRemoverArm(RightY);
                } else if (RightY < -0.1) {
                    m_algaeSubsystem.setRemoverArm(RightY);
                }
            },
            {&m_algaeSubsystem}
        )
    );
    // drivetrain.SetDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     drivetrain.ApplyRequest([this]() -> auto&& {
    //       units::volt_t value{(1 - 0.25) * DriveStick.GetLeftTriggerAxis() + 0.25};
    //       units::volt_t outputMult = filter.Calculate(value);
        

    //         return drive.WithVelocityX(-DriveStick.GetLeftY() * MaxSpeed * outputMult.value()) // Drive forward with positive Y (forward)
    //             .WithVelocityY(-DriveStick.GetLeftX() * MaxSpeed * outputMult.value()) // Drive left with positive X (left)
    //             .WithRotationalRate(-DriveStick.GetRightX() * MaxAngularRate * outputMult.value()); // Drive counterclockwise with negative X (left)
    //     })
    // );

    // DriveStick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    // DriveStick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    //     return point.WithModuleDirection(frc::Rotation2d{-DriveStick.GetLeftY(), -DriveStick.GetLeftX()});
    // }));
    // DriveStick.POVUp().WhileTrue(m_coralSubsystem.SetArmAndElevator(L1Angle, L1Height));
    // DriveStick.Back().WhileTrue(frc2::InstantCommand([this]() -> void {
    //      m_coralSubsystem.ResetState();
    //      }).ToPtr());
          
    AuxStick.POVUp().WhileTrue(frc2::InstantCommand([this]() -> void { // L1/Zero Button
          m_coralSubsystem.SetElevator(0);
         }).ToPtr());

    AuxStick.POVRight().WhileTrue(frc2::InstantCommand([this]() -> void { // L2 Button
            m_coralSubsystem.SetElevator(9 + GravityoffsetIn);
        }).ToPtr());

    AuxStick.POVDown().WhileTrue(frc2::InstantCommand([this]() -> void { // L3 Button
            m_coralSubsystem.SetElevator(25 + GravityoffsetIn);
        }).ToPtr());

    AuxStick.POVLeft().WhileTrue(frc2::InstantCommand([this]() -> void { // L4 Button
            m_coralSubsystem.SetElevator(50.5 + GravityoffsetIn);
        }).ToPtr());

    AuxStick.RightTrigger().WhileTrue(frc2::RunCommand([this]() -> void { // Manual Elevator up
            m_coralSubsystem.ManualElevator(1);
        }).ToPtr());

    AuxStick.LeftTrigger().WhileTrue(frc2::RunCommand([this]() -> void { // Manual Elevator down
            m_coralSubsystem.ManualElevator(-0.8);
        }).ToPtr());
    
    // AuxStick.B().ToggleOnTrue(frc2::InstantCommand([this]() -> void { //Algae in Button on
    //     //algae in go
    //     m_algaeSubsystem.setAlgaeIntakeMotors(0.3);
    //      }).ToPtr());

    // AuxStick.B().ToggleOnFalse(frc2::InstantCommand([this]() -> void { //Algae in Button off
    //     //algae in stop
    //     m_algaeSubsystem.setAlgaeIntakeMotors(0);
    //     }).ToPtr());
         
    // AuxStick.X().ToggleOnTrue(frc2::InstantCommand([this]() -> void { //Algae out Button on
    //     //algae out go
    //     m_algaeSubsystem.setAlgaeIntakeMotors(-0.3);
    //      }).ToPtr());

    // AuxStick.X().ToggleOnFalse(frc2::InstantCommand([this]() -> void { //Algae out Button off
    //     //algae out stop
    //     m_algaeSubsystem.setAlgaeIntakeMotors(0);
    //      }).ToPtr());

    AuxStick.Y().WhileTrue(frc2::InstantCommand([this]() -> void {
        //intake preset
        m_coralSubsystem.SetElevator(0);
        m_coralSubsystem.SetIntakeMotors(0.3);
    }).ToPtr());

    AuxStick.A().WhileTrue(frc2::InstantCommand([this]() -> void { // Intake Button and Place on
            m_coralSubsystem.SetIntakeMotors(0.3);
        }).ToPtr());

    AuxStick.A().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Intake Button off
            m_coralSubsystem.SetIntakeMotors(0);
        }).ToPtr());
    
    AuxStick.B().ToggleOnTrue(frc2::InstantCommand([this]() -> void { // Eject Button on
            m_coralSubsystem.SetIntakeMotors(-0.3);
        }).ToPtr());

    AuxStick.B().ToggleOnFalse(frc2::InstantCommand([this]() -> void { // Eject Button off
            m_coralSubsystem.SetIntakeMotors(0);
        }).ToPtr());

    AuxStick.X().ToggleOnTrue(frc2::InstantCommand([this]() -> void {
            m_algaeSubsystem.setAlgaeArm(AlgaeArmMax);
        }, {&m_algaeSubsystem}).ToPtr());

    AuxStick.X().ToggleOnFalse(frc2::InstantCommand([this]() -> void {
            m_algaeSubsystem.setAlgaeArm(AlgaeArmMin);
        }, {&m_algaeSubsystem}).ToPtr());

    AuxStick.LeftBumper().ToggleOnTrue(frc2::InstantCommand([this]() -> void {
            m_algaeSubsystem.setRemoverArm(FlipperMax);
        }, {&m_algaeSubsystem}).ToPtr());

    AuxStick.LeftBumper().ToggleOnFalse(frc2::InstantCommand([this]() -> void {
            m_algaeSubsystem.setRemoverArm(FlipperMin);
        }, {&m_algaeSubsystem}).ToPtr());
    
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