// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"

CoralSubsystem::CoralSubsystem() = default;

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {
    // Update Sensors

    switch (_state) {

        case EMPTY:
            // code
            // claw ready to grab coral
            // if funnel BB == true (change state to CORAL_IN_FUNNEL)
            break;

        case CORAL_IN_FUNNEL:
            // code
            // if trough BB == true (change state to CORAL_IN_TROUGH)
            break;

        case CORAL_IN_TROUGH:
            // code
            // if claw BB == true (change state to ALLOW_CORAL_MOVE)
            break;

        case ALLOW_CORAL_MOVE:
            // code
            // change lights
            // coral_place == true ?? (change state to CORAL_PLACE)
            break;

        case CORAL_PLACE:
            // code
            // arm swing down to place coral
            // if claw BB == false && armPose == lowered (change state to EMPTY)
            break;

        default:
            _state = EMPTY;
    }
}
