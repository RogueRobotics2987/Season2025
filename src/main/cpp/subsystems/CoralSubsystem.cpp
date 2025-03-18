// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "subsystems/CoralSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

CoralSubsystem::CoralSubsystem(LightSubsystem &lights): _light{lights}{

    SparkMaxConfig _elevatorLeaderConfig;
    SparkMaxConfig _elevatorFollowerConfig;
    SparkMaxConfig _intakeTopConfig;
    SparkMaxConfig _algyArmConfig;
    SparkMaxConfig _funnelPinConfig;

    _elevatorFollowerConfig.Follow(_elevatorLeader);

    _elevatorLeaderConfig.encoder.PositionConversionFactor(2.2167).VelocityConversionFactor(1); // 0, 16 inches | 12.857, 44.5 inches | 2.2167 conversation
    _elevatorFollowerConfig.encoder.PositionConversionFactor(2.2167).VelocityConversionFactor(1);
    _intakeTopConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _algyArmConfig.absoluteEncoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _funnelPinConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    
    _funnelPinConfig.SmartCurrentLimit(50);
    _elevatorLeaderConfig.SmartCurrentLimit(50);
    _elevatorFollowerConfig.SmartCurrentLimit(50);
    _intakeTopConfig.SmartCurrentLimit(50);
    _algyArmConfig.SmartCurrentLimit(50);

    _elevatorLeaderConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.03) // 0.01
      .I(0) // .I(0.000005)
      .D(0)
      .OutputRange(-0.3, 1)
      ;

    _elevatorFollowerConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.03)
      .I(0)
      .D(0)
      .OutputRange(-0.3, 1);

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


    _algyArmConfig.closedLoop
       .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
       .P(2)
       .I(0)
       .D(0)
       .OutputRange(-1, 1);
     
     _funnelPinConfig.closedLoop
       .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
       // Set PID values for position control. We don't need to pass a closed
       // loop slot, as it will default to slot 0.
       .P(0.5)
       .I(0)
       .D(0)
       .OutputRange(-1, 1)
       // Set PID values for velocity control in slot 1
       .P(0.0001, ClosedLoopSlot::kSlot1)
       .I(0, ClosedLoopSlot::kSlot1)
       .D(0, ClosedLoopSlot::kSlot1)
       .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
       .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);

    _elevatorLeader.Configure(_elevatorLeaderConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _elevatorFollower.Configure(_elevatorFollowerConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _intakeTop.Configure(_intakeTopConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _algyArm.Configure(_algyArmConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _funnelPin.Configure(_funnelPinConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    
    SetFunnelPin(0);
    SetAlgyArm(0.38);
} 

void CoralSubsystem::SetFunnelPin(double funnelPinSpeed){
    frc::SmartDashboard::PutNumber("Funnel Pin Speed: ", funnelPinSpeed);
    _funnelPin.Set(funnelPinSpeed);
}
void CoralSubsystem::SetIntakeMotors(double intakeSpeed){
    _intakeTop.Set(-intakeSpeed);
    // _intakeRight.Set(intakeSpeed);
}

void CoralSubsystem::SetAlgyArm(double algyDesiredPoint){
    frc::SmartDashboard::PutNumber("Algy Pose: ", algySetPoint);
    algySetPoint = algyDesiredPoint;
    _AlgyArmClosedLoopController.SetReference(algySetPoint, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
}

void CoralSubsystem::SetAlgyArmManual(double algyPoseStepSize){
    algySetPoint = algySetPoint + algyPoseStepSize;

    if (algySetPoint > 0.35) {
        algySetPoint = 0.35;
    }

    frc::SmartDashboard::PutNumber("Algy Pose: ", algySetPoint);
     _AlgyArmClosedLoopController.SetReference(algySetPoint, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
}

// void CoralSubsystem::SetDesiredElevatorheight(double setElevatorHeight){
//     _desiredElevatorHeight = setElevatorHeight;
// }

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

void CoralSubsystem::ManualElevator(double increaseHeight){
    elevatorTotal = elevatorTotal + increaseHeight;

    if(elevatorTotal < 0){
        elevatorTotal = 0;
    }

    _elevatorLeaderClosedLoopController.SetReference(elevatorTotal, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
}
 
 // This method will be called once per scheduler run
void CoralSubsystem::Periodic() { // TODO: should drivers be able to override evelator and arm all the time?
    frc::SmartDashboard::PutString("Periodic Running", "true");        
    frc::SmartDashboard::PutBoolean("_clawBB", _clawBB.Get()); 
    frc::SmartDashboard::PutNumber("_state", _state); 


    // _funnelBB = frc::SmartDashboard::GetBoolean("Funnel Beam Break", false);
    // _troughBB = frc::SmartDashboard::GetBoolean("Trough Beam Break", false);
    // _clawBB = frc::SmartDashboard::GetBoolean("Claw Beam Break", false);
    // _coralPlace = frc::SmartDashboard::GetBoolean("Coral Place", false);

    frc::SmartDashboard::PutBoolean("ClawBB: ", _clawBB.Get());

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

            _light.LightsOff();

            if (!_clawBB.Get()){
                frc::SmartDashboard::PutNumber("_state", _state);
                if (_intakeDelayCount >= 4) {
                    _intakeTop.Set(0);
                    _intakeDelayCount = 0;

            if (!_clawBB.Get()) {
                // turn on intake


                if (_clawBB.Get()){       //while troughBB = true, if clawBB becomes true then the light1 turns off and
                    _state = YES_CORAL;
                }

                _intakeDelayCount++;
            }

            break;

        case YES_CORAL:

            _light.RBSwap();
            
            if(_clawBB.Get()){
                // turn intake off
                coralLoaded = false;
                coralPlace = false;
                _state = NO_CORAL;
            }
            break;

        default:
            _state = NO_CORAL;
    }}
    frc::SmartDashboard::PutNumber("Current Coral State: ", _state);
    frc::SmartDashboard::PutNumber("Current Elevator Level: ", ElevatorLevel);
}}

// frc2::CommandPtr CoralSubsystem::SetElevatorLevelCommand(int DesiredLevel){
//     return this->RunOnce(
//         [this, DesiredLevel] {ElevatorLevel = DesiredLevel;}
//     );
// }       