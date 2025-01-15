// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"

CoralSubsystem::CoralSubsystem() = default;

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {
    switch (_state) {

        case EMPTY:
            //code
            break;

        case CORAL_IN_FUNNEL:
            //code
            break;

        case CORAL_IN_TROUGH:
            //code
            break;

        case ALLOW_CORAL_MOVE:
            //code
            break;

        case CORAL_PLACE:
            //code
            break;

        default:
            _state = EMPTY;
    }
}
