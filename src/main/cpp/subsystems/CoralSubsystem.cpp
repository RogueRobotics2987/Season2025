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


    // TODO: reconsider using a state machine
    switch (_state) {

        case ZERO:
            // set motor encoders to 0
            _elevatorLeader.GetEncoder().SetPosition(0);
            _elevatorFollower.GetEncoder().SetPosition(0);
            _state = NO_CORAL;

            break;

        case NO_CORAL:

            if (_troughBB == true) {
                // turn on intake
                _light2.Set(false);
                _light1.Set(true);
                if (_clawBB == true){       //while troughBB = true, if clawBB becomes true then the light1 turns off and
                    _light1.Set(false);     // it goes to the state "FULL"
                    _state = YES_CORAL;
                }
                _state = YES_CORAL;
            }
            break;

        case YES_CORAL:

            if(_troughBB == false){
                // turn intake off
                _state = NO_CORAL;
                _light1.Set(false);
                _light2.Set(true);
            }
            break;

        default:
            _state = NO_CORAL;
    }
    frc::SmartDashboard::PutNumber("Current Coral State: ", _state);
    frc::SmartDashboard::PutNumber("Current Elevator Level: ", ElevatorLevel);
}

frc2::CommandPtr CoralSubsystem::SetElevatorLevelCommand(int DesiredLevel){
    return this->RunOnce(
        [this, DesiredLevel] {ElevatorLevel = DesiredLevel;}
    );
}       