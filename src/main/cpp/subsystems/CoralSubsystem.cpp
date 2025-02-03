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

    _funnelBB = frc::SmartDashboard::SetDefaultBoolean("Funnel Beam Break", false);
    _troughBB = frc::SmartDashboard::SetDefaultBoolean("Trough Beam Break", false);
    _coralPlace = frc::SmartDashboard::SetDefaultBoolean("Coral Place", false);
    _clawBB = frc::SmartDashboard::SetDefaultBoolean("Claw Beam Break", false);
} 

void CoralSubsystem::Set_coralPlace(bool setCoralPlace) {
    _coralPlace = setCoralPlace;
}

void CoralSubsystem::ResetState(){
    _state = EMPTY;
}

void CoralSubsystem::Set_armAndElevator(double setArmAngle, double setElevatorHeight) {
    double elevatorFirstStageHeight;
    double elevatorSecondStageHeight;
    double heightError;

    if (setElevatorHeight > maxElevatorHeight) { // checking if set point doesnt go over max height
        setElevatorHeight = maxElevatorHeight;
    }
    else if (setElevatorHeight < firstStageMinElevatorHeight) { // checking if set point doesnt go under min height
        setElevatorHeight = firstStageMinElevatorHeight;
    }

    if (setElevatorHeight < safetyElevatorHeight) { // checking if the set point doesnt go under intake pose
        if (setArmAngle < safetyArmAngle) {
            setArmAngle = safetyArmAngle; // TODO: if elevator at 8 inches is this still safe? ANSWER: yes it is safe
        }
    }

    if (setElevatorHeight <= secondStageMaxElevatorHeight) { // calculates the height of the elevator when below 2nd stage max height
        elevatorFirstStageHeight = firstStageMinElevatorHeight;
        elevatorSecondStageHeight = setElevatorHeight - firstStageMinElevatorHeight;
    }
    else if (setElevatorHeight > secondStageMaxElevatorHeight) { // calculates the height of the elevator when above 2nd stage max height
        elevatorSecondStageHeight = secondStageMaxElevatorHeight - firstStageMinElevatorHeight;
        heightError = setElevatorHeight - elevatorSecondStageHeight;
        elevatorFirstStageHeight = heightError;
    }

    if (setArmAngle > maxArmAngle) { // checking if set point doesnt go over max arm angle
        setArmAngle = maxArmAngle;
    }
    else if (setArmAngle < minArmAngle) { // checking if set point doesnt go over min arm angle
        setArmAngle = minArmAngle;
    }

    frc::SmartDashboard::PutNumber("Elevator First Stage SetPoint: ", elevatorFirstStageHeight);
    frc::SmartDashboard::PutNumber("Elevator Second Stage SetPoint: ", elevatorSecondStageHeight);
    frc::SmartDashboard::PutNumber("Arm Angle SetPoint: ", setArmAngle);

    _elevatorLeaderFirstStageClosedLoopController.SetReference(elevatorFirstStageHeight, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
    _elevatorSecondStageClosedLoopController.SetReference(elevatorSecondStageHeight, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
    _grabberArmclosedLoopController.SetReference(setArmAngle, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
    
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Periodic Running", "true");
    // Update Sensors
    // Gets the value of the digital input.  Returns true if the circuit is open.
/*    _funnelBB = _funnelSensor.Get();
    _troughBB = _troughSensor.Get();
    _clawBB = _clawSensor.Get();*/
    _funnelBB = frc::SmartDashboard::GetBoolean("Funnel Beam Break", false);
    _troughBB = frc::SmartDashboard::GetBoolean("Trough Beam Break", false);
    _clawBB = frc::SmartDashboard::GetBoolean("Claw Beam Break", false);
    _coralPlace = frc::SmartDashboard::GetBoolean("Coral Place", false);

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
            Set_armAndElevator(restingArmAngle, restingElevatorHeight);

            if (_troughBB == true) {
                _state = CORAL_IN_TROUGH;
            }
            break;

        // case CORAL_IN_FUNNEL:
        //     // wait until coral is in trough
        //     if (_troughBB == true) {
        //         _state = CORAL_IN_TROUGH;
        //     }
        //     break;

        case CORAL_IN_TROUGH:
            // TODO: Intake auto on button press
                // lower arm and turn on intake motors
                // lower elevator to pick up position
                // turn on both claw motors

            if (_clawBB == true) {
                _state = ALLOW_CORAL_MOVE;
            }
            break;

        case ALLOW_CORAL_MOVE:
            // change lights
            // allow it to move using presets
            // let the drivers do what they want
            if (_coralPlace == true) {
                _state = CORAL_PLACE;
            }
            break;

        case CORAL_PLACE:
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
