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

using namespace rev::spark;
using namespace CoralSubsystemConstants;

enum PossibleStates {
  ZERO,
  NO_CORAL,
  YES_CORAL
 };



class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem();

  // these's are the functions we use
  void SetElevator(double setElevator);
  void SetIntakeMotors(double intakeSpeed);
  void IncrementOffsets(double offsetElevator);
  void ManualElevator(double increaseHeight);
  void SetAlgyArm(double setAlgyArm);

  frc2::CommandPtr SetElevatorLevelCommand(int DesiredLevel);
  double GetDesiredElevatorHeight();
  double GetDesiredArmAngle();
  void LightsOff();
  void RBSwap();
  void LightsPink();
  void LightsCyan();
  void PinkBlink();
  void CyanBlink();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 public:
    enum PossibleStates _state = ZERO;

    int ElevatorLevel = 0;
    double elevatorOffset = 0;
    double elevatorTotal = 0;
    int intakeDelayCount = 0;

    // double climberTotal = 0;

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

    // climber
    // SparkMax _climber{CoralSubsystemConstants::CANIdClimber, SparkMax::MotorType::kBrushless};
    // SparkClosedLoopController _climberClosedLoopController = _climber.GetClosedLoopController();
    // SparkRelativeEncoder _climberencoder = _climber.GetEncoder();

    // // intakeRight
    SparkMax _algyArm{CoralSubsystemConstants::CANIdAlgyArm, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _AlgyArmClosedLoopController = _algyArm.GetClosedLoopController();
    SparkRelativeEncoder _algyArmEncoder = _algyArm.GetEncoder();
    
    // Initializes a DigitalInput on DIO 0
    // frc::DigitalInput _funnelSensor{0};
    // frc::DigitalInput _troughSensor{1};
    frc::DigitalInput _clawBB{3};

    frc::DigitalOutput _light1{4};
    frc::DigitalOutput _light2{5};
    frc::DigitalOutput _light3{6};

    double _elevatorHeight = CoralSubsystemConstants::restingElevatorHeight;
    double _desiredElevatorHeight = restingElevatorHeight;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
