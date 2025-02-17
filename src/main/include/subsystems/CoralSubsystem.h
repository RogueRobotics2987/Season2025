// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>

using namespace rev::spark;
using namespace CoralSubsystemConstants;

enum PossibleStates {
  // START_CALIBRATION,
  // ZERO,
  // EMPTY,
  // // CORAL_IN_FUNNEL,
  // CORAL_IN_TROUGH,
  // ALLOW_CORAL_MOVE,
  // CORAL_PLACE
 };



class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem();

  // these's are the functions we use
  void SetEverything(double setElevator);
  void SetIntakeMotors(double intakeSpeed);
  void IncrementOffsets(double offsetElevator);

  void SetCoralPlace(bool setCoralPlace);
  void ResetState();

  void SetDesiredElevatorheight(double setElevatorHeight);
  void SetElevator();

  frc2::CommandPtr SetElevatorLevelCommand(int DesiredLevel);
  double GetDesiredElevatorHeight();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 public:
    // enum PossibleStates _state = START_CALIBRATION;

    int ElevatorLevel = 0;

    double elevatorOffset = 0;

    double elevatorTotal = 0;

    // the motors on the robot
    
    // elevatorLeft
    SparkMax _elevatorLeaderFirstStage{CoralSubsystemConstants::CANIdLeaderElevatorFirstStage, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _elevatorLeaderFirstStageClosedLoopController = _elevatorLeaderFirstStage.GetClosedLoopController();
    SparkRelativeEncoder _elevatorLeaderFirstStageEncoder = _elevatorLeaderFirstStage.GetEncoder();

    // elevatorRight
    SparkMax _elevatorFollowerFirstStage{CoralSubsystemConstants::CANIdFollowerElevatorFirstStage, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _elevatorFollowerFirstStageClosedLoopController = _elevatorFollowerFirstStage.GetClosedLoopController();
    SparkRelativeEncoder _elevatorFollowerFirstStageEncoder = _elevatorFollowerFirstStage.GetEncoder();

    // elevatorSecondStage
    SparkMax _elevatorSecondStage{CoralSubsystemConstants::CANIdElevatorSecondStage, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _elevatorSecondStageClosedLoopController = _elevatorSecondStage.GetClosedLoopController();
    SparkRelativeEncoder _elevatorSecondStageEncoder = _elevatorSecondStage.GetEncoder();

    // intakeLeft
    SparkMax _intakeLeft{CoralSubsystemConstants::CANIdLeftIntake, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _intakeLeftclosedLoopController = _intakeLeft.GetClosedLoopController(); // TODO: no close loop controllers
    SparkRelativeEncoder _intakeLeftencoder = _intakeLeft.GetEncoder();

    // intakeRight
    SparkMax _intakeRight{CoralSubsystemConstants::CANIdRightIntake, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _intakeRightclosedLoopController = _intakeRight.GetClosedLoopController();
    SparkRelativeEncoder _intakeRightencoder = _intakeRight.GetEncoder();
    
    // Initializes a DigitalInput on DIO 0
    // frc::DigitalInput _funnelSensor{0};
    // frc::DigitalInput _troughSensor{1};
    frc::DigitalInput _clawSensor{0};

    // bool _funnelBB = false;
    // bool _troughBB = false;
    bool _clawBB = false;
    double _elevatorHeight = CoralSubsystemConstants::restingElevatorHeight;

    double _desiredElevatorHeight = restingElevatorHeight;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
