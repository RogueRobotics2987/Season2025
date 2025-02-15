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

  void SetCoralPlace(bool setCoralPlace);
  void ResetState();
  void Set_armAndElevator(double setArmAngle, double setElevatorHeight);
  frc2::CommandPtr SetElevatorLevelCommand(int DesiredLevel);
  void SetIntakeMotors(double intakeSpeed); 
  void SetDesiredArmAngle(double setArmAngle);
  void SetDesiredElevatorheight(double setElevatorHeight);
  void SetDesiredArmAngleAndElevatorHeight(double setArmAngle, double setElevatorheight);
  void SetArmAndElevator();
  void SetEverything(double setArmAngle, double setStageOne, double setStageTwo);
  double GetDesiredElevatorHeight();
  double GetDesiredArmAngle();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
    // enum PossibleStates _state = START_CALIBRATION;

    int ElevatorLevel = 0;


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

    // grabberArm
    SparkMax _grabberArm{CoralSubsystemConstants::CANIdGrabberArm, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _grabberArmclosedLoopController = _grabberArm.GetClosedLoopController();
    SparkAbsoluteEncoder _grabberArmencoder = _grabberArm.GetAbsoluteEncoder();

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
    frc::DigitalInput _troughSensor{1};
    frc::DigitalInput _clawSensor{2};

    // bool _funnelBB = false;
    bool _troughBB = false;
    bool _clawBB = false;
    bool _coralPlace = false;
    double _armAngle = CoralSubsystemConstants::restingArmAngle;
    double _elevatorHeight = CoralSubsystemConstants::restingElevatorHeight;

    double _desiredArmAngle = restingArmAngle;
    double _desiredElevatorHeight = restingElevatorHeight;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
