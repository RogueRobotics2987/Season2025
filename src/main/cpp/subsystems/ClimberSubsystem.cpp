// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include <rev/config/SparkMaxConfig.h>
#include "Constants.h"

ClimberSubsystem::ClimberSubsystem() {

    SparkMaxConfig _climberConfig;

    _climberConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _climberConfig.SmartCurrentLimit(50);

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
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
void ClimberSubsystem::SetClimber(double ClimberSpeed){
    _climber.Set(ClimberSpeed);
}