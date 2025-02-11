// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

CoralSubsystem::CoralSubsystem(){
    SparkMaxConfig _elevatorLeaderFirstStageConfig;
    SparkMaxConfig _elevatorFollowerFirstStageConfig;
    SparkMaxConfig _elevatorSecondStageConfig;
    SparkMaxConfig _grabberArmConfig;
    SparkMaxConfig _intakeLeftConfig;
    SparkMaxConfig _intakeRightConfig;

    _elevatorFollowerFirstStageConfig.Follow(_elevatorLeaderFirstStage);

    _elevatorLeaderFirstStageConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _elevatorFollowerFirstStageConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _elevatorSecondStageConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _grabberArmConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _intakeLeftConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _intakeRightConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

    _elevatorLeaderFirstStageConfig.closedLoop
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

    _elevatorFollowerFirstStageConfig.closedLoop
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

      _elevatorSecondStageConfig.closedLoop
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

    _grabberArmConfig.closedLoop
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

    _intakeLeftConfig.closedLoop
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

    _intakeRightConfig.closedLoop
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


    _elevatorLeaderFirstStage.Configure(_elevatorLeaderFirstStageConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _elevatorFollowerFirstStage.Configure(_elevatorFollowerFirstStageConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _elevatorSecondStage.Configure(_elevatorSecondStageConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _grabberArm.Configure(_grabberArmConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _intakeLeft.Configure(_intakeLeftConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _intakeRight.Configure(_intakeRightConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    // _funnelBB = frc::SmartDashboard::SetDefaultBoolean("Funnel Beam Break", false);
    _troughBB = frc::SmartDashboard::SetDefaultBoolean("Trough Beam Break", false);
    _coralPlace = frc::SmartDashboard::SetDefaultBoolean("Coral Place", false);
    _clawBB = frc::SmartDashboard::SetDefaultBoolean("Claw Beam Break", false);
} 

void CoralSubsystem::SetCoralPlace(bool setCoralPlace) {
    _coralPlace = setCoralPlace;
    SetArmAndElevator();
}

void CoralSubsystem::ResetState(){
    _state = EMPTY;
}

void CoralSubsystem::SetDesiredArmAngle(double setArmAngle){
    _desiredArmAngle = setArmAngle;
    SetArmAndElevator();
}

void CoralSubsystem::SetDesiredArmAngleAndElevatorHeight(double setArmAngle, double setElevatorHeight){
    SetDesiredArmAngle(setArmAngle);
    SetDesiredElevatorheight(setElevatorHeight);
}

void CoralSubsystem::SetDesiredElevatorheight(double setElevatorHeight){
    _desiredElevatorHeight = setElevatorHeight;
}

double CoralSubsystem::GetDesiredElevatorHeight(){
    return _desiredElevatorHeight;
}

double CoralSubsystem::GetDesiredArmAngle(){
    return _desiredArmAngle;
}

void CoralSubsystem::SetIntakeMotors(double intakeSpeed){
    _intakeLeft.Set(intakeSpeed);
    _intakeRight.Set(intakeSpeed);
}

void CoralSubsystem::SetArmAndElevator() {
    double elevatorFirstStageHeight;
    double elevatorSecondStageHeight;
    double heightError;

    if (_desiredElevatorHeight > maxElevatorHeight) { // checking if set point doesnt go over max height
        _desiredElevatorHeight = maxElevatorHeight;
    }
    else if (_desiredElevatorHeight < firstStageMinElevatorHeight) { // checking if set point doesnt go under min height
        _desiredElevatorHeight = firstStageMinElevatorHeight;
    }

    if (_desiredElevatorHeight < safetyElevatorHeight) { // checking if the set point doesnt go under intake pose
        if (_desiredArmAngle < safetyArmAngle) {
            _desiredArmAngle= safetyArmAngle; // TODO: if elevator at 8 inches is this still safe? ANSWER: yes it is safe
        }
    }

    if (_desiredElevatorHeight < firstStageMinElevatorHeight) { // sets the arm angle to 90 if elevator is lower than 8 inches
        _grabberArm.Set(90);
    }

    if (_desiredElevatorHeight <= secondStageMaxElevatorHeight) { // calculates the height of the elevator when below 2nd stage max height
        elevatorFirstStageHeight = firstStageMinElevatorHeight;
        elevatorSecondStageHeight = _desiredElevatorHeight - firstStageMinElevatorHeight;
    }
    else if (_desiredElevatorHeight > secondStageMaxElevatorHeight) { // calculates the height of the elevator when above 2nd stage max height
        elevatorSecondStageHeight = secondStageMaxElevatorHeight - firstStageMinElevatorHeight;
        heightError = _desiredElevatorHeight - elevatorSecondStageHeight;
        elevatorFirstStageHeight = heightError;
    }

    if (_desiredArmAngle > maxArmAngle) { // checking if set point doesnt go over max arm angle
        _desiredArmAngle = maxArmAngle;
    }
    else if (_desiredArmAngle < minArmAngle) { // checking if set point doesnt go over min arm angle
        _desiredArmAngle = minArmAngle;
    }

    if (_desiredArmAngle < maxArmAngle && _desiredArmAngle > minArmAngle){
        _grabberArm.Set(_desiredArmAngle);
    }

    frc::SmartDashboard::PutNumber("Elevator First Stage SetPoint: ", elevatorFirstStageHeight);
    frc::SmartDashboard::PutNumber("Elevator Second Stage SetPoint: ", elevatorSecondStageHeight);
    frc::SmartDashboard::PutNumber("Arm Angle SetPoint: ", _desiredArmAngle);

    _elevatorLeaderFirstStageClosedLoopController.SetReference(elevatorFirstStageHeight, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
    _elevatorSecondStageClosedLoopController.SetReference(elevatorSecondStageHeight, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
    _grabberArmclosedLoopController.SetReference(_desiredArmAngle, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
    
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() { // TODO: should drivers be able to override evelator and arm all the time?
    frc::SmartDashboard::PutString("Periodic Running", "true");
    // Update Sensors
    // Gets the value of the digital input.  Returns true if the circuit is open.

    // TODO: is this code needed?
    //_funnelBB = _funnelSensor.Get();
    _troughBB = _troughSensor.Get();
    _clawBB = _clawSensor.Get();

    // _funnelBB = frc::SmartDashboard::GetBoolean("Funnel Beam Break", false);
    _troughBB = frc::SmartDashboard::GetBoolean("Trough Beam Break", false);
    _clawBB = frc::SmartDashboard::GetBoolean("Claw Beam Break", false);
    // _coralPlace = frc::SmartDashboard::GetBoolean("Coral Place", false);

    switch (_state) {

        case START_CALIBRATION:
            // lower elevator at constant speed
            _elevatorLeaderFirstStage.Set(CoralSubsystemConstants::elevatorZeroReverseSpeed);
            _elevatorSecondStage.Set(CoralSubsystemConstants::elevatorZeroReverseSpeed);

            if (_elevatorLeaderFirstStage.GetReverseLimitSwitch().Get() && _elevatorSecondStage.GetReverseLimitSwitch().Get()) {
                _state = ZERO;
            }
            break;

        case ZERO:
            // set motor encoders to 0
            _elevatorLeaderFirstStage.GetEncoder().SetPosition(0);
            _elevatorFollowerFirstStage.GetEncoder().SetPosition(0);
            _elevatorSecondStage.GetEncoder().SetPosition(0);
            _state = EMPTY;

            break;

        case EMPTY:
            // claw ready to grab coral
            // elevator at loading position
            // arm at loading position
            // both claw motors off
            SetDesiredArmAngleAndElevatorHeight(restingArmAngle, restingElevatorHeight);

            if (_troughBB == true) {
                _state = CORAL_IN_TROUGH;
            }
            break;

        // this state is not being used for now
        // case CORAL_IN_FUNNEL:
        //     // wait until coral is in trough
        //     if (_troughBB == true) {
        //         _state = CORAL_IN_TROUGH;
        //     }
        //     break;

        case CORAL_IN_TROUGH:
            // auto intake:
                // lower arm and turn on intake motors
                // lower elevator to pick up position
                // turn on both claw motors
            SetIntakeMotors(intakeSpeed);
            SetDesiredArmAngleAndElevatorHeight(restingArmAngle, intakeHeight);

            if (_clawBB == true) { 
                SetIntakeMotors(intakeOff);
                SetDesiredArmAngleAndElevatorHeight(restingArmAngle, restingElevatorHeight);
                _state = ALLOW_CORAL_MOVE;
            }
            break;

        case ALLOW_CORAL_MOVE:
            // change lights
            // allow it to move using presets
            // let the drivers do what they want
            // allow drives to move it manually

            // these numbers will be used for preset elevator heights (these numbers will be changed these numbers are in meters)
            // L1 height for elevator = 0.74
            // L2 height for elevator = 1.07
            // L3 height for elevator = 1.24
            // L4 height for elevator = 1.42

            if (_coralPlace == true) {
                _state = CORAL_PLACE;
            }
            break;

        case CORAL_PLACE:

            if(_clawBB == true && _grabberArmencoder.GetPosition() <= armLowered) {
                SetDesiredArmAngle(placingArmAngle);
                _state = ALLOW_CORAL_MOVE;
            }

            if (_clawBB == false && _grabberArmencoder.GetPosition() <= armLowered) {
                _state = EMPTY;
            }
            break;

        default:
            _state = EMPTY;
    }

    frc::SmartDashboard::PutNumber("Current Coral State: ", _state);
    frc::SmartDashboard::PutNumber("Current Elevator Level: ", ElevatorLevel);
}

frc2::CommandPtr CoralSubsystem::SetElevatorLevelCommand(int DesiredLevel){
    return this->RunOnce(
        [this, DesiredLevel] {ElevatorLevel = DesiredLevel;}
    );
}