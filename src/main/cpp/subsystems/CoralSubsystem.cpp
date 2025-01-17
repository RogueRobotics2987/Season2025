// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

CoralSubsystem::CoralSubsystem() = default;

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
            if (_funnelBB == true) {
                _state = CORAL_IN_FUNNEL;
            }
            break;

        case CORAL_IN_FUNNEL:
            // code
            if (_troughBB == true) {
                _state = CORAL_IN_TROUGH;
            }
            break;

        case CORAL_IN_TROUGH:
            // code
            // lower arm and turn on intake motors
            if (_clawBB == true) {
                _state = ALLOW_CORAL_MOVE;
            }
            break;

        case ALLOW_CORAL_MOVE:
            // code
            // change lights
            // allow it to move using presets
            if (_coralPlace == true) {
                _state = CORAL_PLACE;
            }
            break;

        case CORAL_PLACE:
            // code
            // arm swings down to place coral
            // if claw BB == false && armPose == lowered (change state to EMPTY)
            if (_clawBB == false) { // TODO: check arm angle
                _state = EMPTY;
            }
            break;

        default:
            _state = EMPTY;
    }

    frc::SmartDashboard::PutNumber("Current Coral State: ", _state);
}
