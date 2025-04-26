// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/LightSubsystem.h"

using namespace rev::spark;
using namespace CoralSubsystemConstants;




class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem();

  // these's are the functions we use
  void SetElevator(double setElevator);
  void SetIntakeMotors(double intakeSpeed);
  void IncrementOffsets(double offsetElevator);
  void ManualElevator(double increaseHeight);
  void SetAlgyArm(double algyPose);
  void SetAlgyArmManual(double algyPoseStepSize);
  void SetFunnelPin(double funnelPinSpeed);
  bool GetBB();

  frc2::CommandPtr SetElevatorLevelCommand(int DesiredLevel);
  double GetDesiredElevatorHeight();
  double GetDesiredArmAngle();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 public:

    int ElevatorLevel = 0;
    double elevatorOffset = 0;
    double elevatorTotal = 0;
    double algyPose = 0.35;
    double algySetPoint = 0.35;

    // the motors on the robot
    
    // elevatorLeft
    SparkMax _elevatorLeader{CoralSubsystemConstants::CANIdLeaderElevator, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _elevatorLeaderClosedLoopController = _elevatorLeader.GetClosedLoopController();
    SparkRelativeEncoder _elevatorLeaderEncoder = _elevatorLeader.GetEncoder();

    // elevatorRight
    SparkMax _elevatorFollower{CoralSubsystemConstants::CANIdFollowerElevator, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _elevatorFollowerClosedLoopController = _elevatorFollower.GetClosedLoopController();
    SparkRelativeEncoder _elevatorFollowerEncoder = _elevatorFollower.GetEncoder();

    // intakeTop
    SparkMax _intakeTop{CoralSubsystemConstants::CANIdTopIntake, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _intakeTopClosedLoopController = _intakeTop.GetClosedLoopController(); // TODO: no close loop controllers
    SparkRelativeEncoder _intakeTopEncoder = _intakeTop.GetEncoder();

    SparkMax _algyArm{CoralSubsystemConstants::CANIdAlgyArm, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _AlgyArmClosedLoopController = _algyArm.GetClosedLoopController();
    SparkAbsoluteEncoder _algyArmEncoder = _algyArm.GetAbsoluteEncoder();
    
    SparkMax _funnelPin{CoralSubsystemConstants::CANIdFunnelPin, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _funnelPinclosedLoopController = _funnelPin.GetClosedLoopController();
    SparkRelativeEncoder _funnelPinencoder = _funnelPin.GetEncoder();
    
    // Initializes a DigitalInput on DIO 0
    // frc::DigitalInput _funnelSensor{0};
    // frc::DigitalInput _troughSensor{1};
    frc::DigitalInput _clawBB{3};



    double _elevatorHeight = CoralSubsystemConstants::restingElevatorHeight;
    double _desiredElevatorHeight = restingElevatorHeight;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  private:
};