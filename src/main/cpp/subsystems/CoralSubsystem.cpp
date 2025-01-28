// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

CoralSubsystem::CoralSubsystem(){
    SparkMaxConfig _elevatorLeftConfig;
    SparkMaxConfig _elevatorRightConfig;
    SparkMaxConfig _grabberArmConfig;
    SparkMaxConfig _intakeLeftConfig;
    SparkMaxConfig _intakeRightConfig;

    _elevatorLeftConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _elevatorRightConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _grabberArmConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _intakeLeftConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _intakeRightConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

    _elevatorLeftConfig.closedLoop
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

    _elevatorRightConfig.closedLoop
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


    _elevatorLeft.Configure(_elevatorLeftConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _elevatorRight.Configure(_elevatorRightConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _grabberArm.Configure(_grabberArmConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _intakeLeft.Configure(_intakeLeftConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _intakeRight.Configure(_intakeRightConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
} 

void CoralSubsystem::Set_coralPlace(bool setCoralPlace) {
    _coralPlace = setCoralPlace;
}

void CoralSubsystem::ResetState(){
    _state = EMPTY;
}

void CoralSubsystem::Set_armAndElevator(double setArmAngle, double setElevatorHeight) {
    _armAngle = setArmAngle;
    _elevatorHeight = setElevatorHeight;
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {

    // Update Sensors
    // Gets the value of the digital input.  Returns true if the circuit is open.
    _funnelBB = _funnelSensor.Get();
    _troughBB = _troughSensor.Get();
    _clawBB = _clawSensor.Get();
    
    switch (_state) {

        case EMPTY:
            // code
            // claw ready to grab coral
            // elevator at loading position
            // arm at loading position
            // both claw motors off

            _elavatorLeft.GetPIDController()

            if (_funnelBB == true) {
                _state = CORAL_IN_FUNNEL;
            }
            break;

        case CORAL_IN_FUNNEL:
            // code
            // wait until coral is in trough
            if (_troughBB == true) {
                _state = CORAL_IN_TROUGH;
            }
            break;

        case CORAL_IN_TROUGH:
            // code
            // lower arm and turn on intake motors
            // lower elevator to pick up position
            // turn on both claw motors
            if (_clawBB == true) {
                _state = ALLOW_CORAL_MOVE;
            }
            break;

        case ALLOW_CORAL_MOVE:
            // code
            // change lights
            // allow it to move using presets
            // let the drivers do what they want
            if (_coralPlace == true) {
                _state = CORAL_PLACE;
            }
            break;

        case CORAL_PLACE:
            // code
            // arm swings down to place coral
            // if claw BB == false && armPose == lowered (change state to EMPTY)
            // arm move to place angle
            if (_clawBB == false) { // TODO: check arm angle
                _state = EMPTY;
            }
            break;

        default:
            _state = EMPTY;
    }

    frc::SmartDashboard::PutNumber("Current Coral State: ", _state);
}
