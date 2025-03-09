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
    SparkMaxConfig _climberConfig;
    // SparkMaxConfig _algyArmConfig;

    _elevatorFollowerConfig.Follow(_elevatorLeader);

    _elevatorLeaderConfig.encoder.PositionConversionFactor(2.2167).VelocityConversionFactor(1); // 0, 16 inches | 12.857, 44.5 inches | 2.2167 conversation
    _elevatorFollowerConfig.encoder.PositionConversionFactor(2.2167).VelocityConversionFactor(1);
    _intakeTopConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _climberConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    // _algyArmConfig.absoluteEncoder.PositionConversionFactor(1).VelocityConversionFactor(1);

    _elevatorLeaderConfig.SmartCurrentLimit(50);
    _elevatorFollowerConfig.SmartCurrentLimit(50);
    _intakeTopConfig.SmartCurrentLimit(50);
    _climberConfig.SmartCurrentLimit(50);
    // _algyArmConfig.SmartCurrentLimit(50);

    _elevatorLeaderConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.02) // 0.01
      .I(0) // .I(0.000005)
      .D(0)
      .OutputRange(-0.2, 1)
      ;

    _elevatorFollowerConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.02)
      .I(0)
      .D(0)
      .OutputRange(-0.2, 1);

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

      _climberConfig.closedLoop
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

    // _algyArmConfig.closedLoop
    //   .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
    //   // Set PID values for position control. We don't need to pass a closed
    //   // loop slot, as it will default to slot 0.
    //   .P(0.01)
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
    // _algyArm.Configure(_algyArmConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    // _funnelBB = frc::SmartDashboard::SetDefaultBoolean("Funnel Beam Break", false);
    // _clawBB = frc::SmartDashboard::SetDefaultBoolean("Claw Beam Break", false);
} 

void CoralSubsystem::SetIntakeMotors(double intakeSpeed){
    _intakeTop.Set(-intakeSpeed);
    // _intakeRight.Set(intakeSpeed);
}

// void CoralSubsystem::SetAlgyArm(double setAlgyArm){
//     _AlgyArmClosedLoopController.SetReference(setAlgyArm, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
// }

// void CoralSubsystem::SetDesiredElevatorheight(double setElevatorHeight){
//     _desiredElevatorHeight = setElevatorHeight;
// }

void CoralSubsystem::SetClimber(double ClimberSpeed){
    _climber.Set(ClimberSpeed);
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
    frc::SmartDashboard::PutBoolean("_light1", _light1.Get());
    frc::SmartDashboard::PutBoolean("_light2", _light2.Get());
    frc::SmartDashboard::PutBoolean("_light3", _light3.Get());

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

            LightsOff();

            if (!_clawBB.Get()){
                _intakeTop.Set(0);
            }

            if (!_clawBB.Get()) {
                // turn on intake
                _light2.Set(false);
                _light1.Set(true);

                if (_clawBB.Get()){       //while troughBB = true, if clawBB becomes true then the light1 turns off and
                    _light1.Set(false);     // it goes to the state "FULL"
                    _state = YES_CORAL;
                }
                
                _state = YES_CORAL;
            }

            break;

        case YES_CORAL:

            RBSwap();
            
            if(_clawBB.Get()){
                // turn intake off
                _state = NO_CORAL;
            }
            break;

        default:
            _state = NO_CORAL;
    }
    frc::SmartDashboard::PutNumber("Current Coral State: ", _state);
    frc::SmartDashboard::PutNumber("Current Elevator Level: ", ElevatorLevel);
}

void CoralSubsystem::LightsOff() {
    _light1.Set(false);
    _light2.Set(false);
    _light3.Set(false);
}
void CoralSubsystem::RBSwap() {
    _light1.Set(true);
    _light2.Set(false);
    _light3.Set(false);
}
void CoralSubsystem::LightsPink() {
    _light1.Set(false);
    _light2.Set(true);
    _light3.Set(false);
}
void CoralSubsystem::LightsCyan() {
    _light1.Set(false);
    _light2.Set(false);
    _light3.Set(true);
}
void CoralSubsystem::PinkBlink() {
    _light1.Set(true);
    _light2.Set(true);
    _light3.Set(false);
}
void CoralSubsystem::CyanBlink(){
    _light1.Set(true);
    _light2.Set(false);
    _light3.Set(true);
}

frc2::CommandPtr CoralSubsystem::SetElevatorLevelCommand(int DesiredLevel){
    return this->RunOnce(
        [this, DesiredLevel] {ElevatorLevel = DesiredLevel;}
    );
}       
