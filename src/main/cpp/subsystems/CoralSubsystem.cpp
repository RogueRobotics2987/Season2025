// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "subsystems/CoralSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

CoralSubsystem::CoralSubsystem(){
    SparkMaxConfig _elevatorLeaderConfig;
    SparkMaxConfig _elevatorFollowerConfig;
    SparkMaxConfig _intakeTopConfig;
    // SparkMaxConfig _intakeRightConfig;

    _elevatorFollowerConfig.Follow(_elevatorLeader);

    _elevatorLeaderConfig.encoder.PositionConversionFactor(2.2167).VelocityConversionFactor(1); // 0, 16 inches | 12.857, 44.5 inches | 2.2167 conversation
    _elevatorFollowerConfig.encoder.PositionConversionFactor(2.2167).VelocityConversionFactor(1);
    _intakeTopConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    // _intakeRightConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

    _elevatorLeaderConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.08)
      .I(0.0000025)
      .D(0)
      .OutputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .P(0.0001, ClosedLoopSlot::kSlot1)
      .I(0, ClosedLoopSlot::kSlot1)
      .D(0, ClosedLoopSlot::kSlot1)
      .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
      .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);

    _elevatorFollowerConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.08)
      .I(0.0000025)
      .D(0)
      .OutputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .P(0.0001, ClosedLoopSlot::kSlot1)
      .I(0, ClosedLoopSlot::kSlot1)
      .D(0, ClosedLoopSlot::kSlot1)
      .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
      .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);

    _intakeTopConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.1)
      .I(0)
      .D(0)
      .OutputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .P(0.0001, ClosedLoopSlot::kSlot1)
      .I(0, ClosedLoopSlot::kSlot1)
      .D(0, ClosedLoopSlot::kSlot1)
      .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
      .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);

    // _intakeRightConfig.closedLoop
    //   .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    //   // Set PID values for position control. We don't need to pass a closed
    //   // loop slot, as it will default to slot 0.
    //   .P(0.1)
    //   .I(0)
    //   .D(0)
    //   .OutputRange(-1, 1)
    //   // Set PID values for velocity control in slot 1
    //   .P(0.0001, ClosedLoopSlot::kSlot1)
    //   .I(0, ClosedLoopSlot::kSlot1)
    //   .D(0, ClosedLoopSlot::kSlot1)
    //   .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
    //   .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);

    _elevatorLeader.Configure(_elevatorLeaderConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _elevatorFollower.Configure(_elevatorFollowerConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _intakeTop.Configure(_intakeTopConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    // _intakeRight.Configure(_intakeRightConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    // _funnelBB = frc::SmartDashboard::SetDefaultBoolean("Funnel Beam Break", false);
    _clawBB = frc::SmartDashboard::SetDefaultBoolean("Claw Beam Break", false);
} 

void CoralSubsystem::SetIntakeMotors(double intakeSpeed){
    _intakeTop.Set(-intakeSpeed);
    // _intakeRight.Set(intakeSpeed);
}

void CoralSubsystem::SetDesiredElevatorheight(double setElevatorHeight){
    _desiredElevatorHeight = setElevatorHeight;
}

double CoralSubsystem::GetDesiredElevatorHeight(){
    return _desiredElevatorHeight;
}

void CoralSubsystem::IncrementOffsets(double offsetElevator){
    double elevatorSetPoint = elevatorTotal + offsetElevator;
    elevatorOffset += offsetElevator;

    // if (elevatorSetPoint > 21.16){
    //     elevatorSetPoint = 21.16;
    // }

    _elevatorLeaderClosedLoopController.SetReference(elevatorSetPoint, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
}

void CoralSubsystem::SetElevator(double setElevator){
    elevatorTotal = setElevator + elevatorOffset;

    // if (elevatorTotal > 21.16){
    //     elevatorTotal = 21.16;
    // }

    _elevatorLeaderClosedLoopController.SetReference(elevatorTotal, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() { // TODO: should drivers be able to override evelator and arm all the time?
//     frc::SmartDashboard::PutString("Periodic Running", "true");
//     // Update Sensors
//     // Gets the value of the digital input.  Returns true if the circuit is open.

//     _clawBB = _clawSensor.Get();

//     // _funnelBB = frc::SmartDashboard::GetBoolean("Funnel Beam Break", false);
//     // _troughBB = frc::SmartDashboard::GetBoolean("Trough Beam Break", false);
//     _clawBB = frc::SmartDashboard::GetBoolean("Claw Beam Break", false);
//     // _coralPlace = frc::SmartDashboard::GetBoolean("Coral Place", false);

    if (_elevatorLeader.GetReverseLimitSwitch().Get()) {
        _elevatorLeader.GetEncoder().SetPosition(0);
        _elevatorFollower.GetEncoder().SetPosition(0);
    }


    // TODO: reconsider using a state machine answer: We are not using the state machine.
    // switch (_state) {

    //     case START_CALIBRATION:
    //         // lower elevator at constant speed
    //         _elevatorLeaderFirstStage.Set(CoralSubsystemConstants::elevatorZeroReverseSpeed);
    //         _elevatorSecondStage.Set(CoralSubsystemConstants::elevatorZeroReverseSpeed);

    //         if (_elevatorLeaderFirstStage.GetReverseLimitSwitch().Get() && _elevatorSecondStage.GetReverseLimitSwitch().Get()) {
    //             _state = ZERO;
    //         }
    //         break;

    //     case ZERO:
    //         // set motor encoders to 0
    //         _elevatorLeaderFirstStage.GetEncoder().SetPosition(0);
    //         _elevatorFollowerFirstStage.GetEncoder().SetPosition(0);
    //         _elevatorSecondStage.GetEncoder().SetPosition(0);
    //         _state = EMPTY;

    //         break;

    //     case EMPTY:
    //         // claw ready to grab coral
    //         // elevator at loading position
    //         // arm at loading position
    //         // both claw motors off
    //         SetDesiredArmAngleAndElevatorHeight(restingArmAngle, restingElevatorHeight);

    //         if (_troughBB == true) {
    //             _state = CORAL_IN_TROUGH;
    //         }
    //         break;

    //     // this state is not being used for now
    //     // case CORAL_IN_FUNNEL:
    //     //     // wait until coral is in trough
    //     //     if (_troughBB == true) {
    //     //         _state = CORAL_IN_TROUGH;
    //     //     }
    //     //     break;

    //     case CORAL_IN_TROUGH:
    //         // auto intake:
    //             // lower arm and turn on intake motors
    //             // lower elevator to pick up position
    //             // turn on both claw motors
    //         SetIntakeMotors(intakeSpeed);
    //         SetDesiredArmAngleAndElevatorHeight(restingArmAngle, intakeHeight);

    //         if (_clawBB == true) { 
    //             SetIntakeMotors(intakeOff);
    //             SetDesiredArmAngleAndElevatorHeight(restingArmAngle, restingElevatorHeight);
    //             _state = ALLOW_CORAL_MOVE;
    //         }
    //         break;

    //     case ALLOW_CORAL_MOVE:
    //         // change lights
    //         // allow it to move using presets
    //         // let the drivers do what they want
    //         // allow drives to move it manually

    //         // these numbers will be used for preset elevator heights (these numbers will be changed these numbers are in meters)
    //         // L1 height for elevator = 0.74
    //         // L2 height for elevator = 1.07
    //         // L3 height for elevator = 1.24
    //         // L4 height for elevator = 1.42

    //         if (_coralPlace == true) {
    //             _state = CORAL_PLACE;
    //         }
    //         break;

    //     case CORAL_PLACE:

    //         if(_clawBB == true && _grabberArmencoder.GetPosition() <= armLowered) {
    //             SetDesiredArmAngle(placingArmAngle);
    //             _state = ALLOW_CORAL_MOVE;
    //         }

    //         if (_clawBB == false && _grabberArmencoder.GetPosition() <= armLowered) {
    //             _state = EMPTY;
    //         }
    //         break;

    //     default:
    //         _state = EMPTY;
    
    // frc::SmartDashboard::PutNumber("Current Coral State: ", _state);
    frc::SmartDashboard::PutNumber("Current Elevator Level: ", elevatorTotal);
    }

// }

frc2::CommandPtr CoralSubsystem::SetElevatorLevelCommand(int DesiredLevel){
    return this->RunOnce(
        [this, DesiredLevel] {ElevatorLevel = DesiredLevel;}
    );
}