// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/AlgaeSubsystem.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

AlgaeSubsystem::AlgaeSubsystem(){
    SparkMaxConfig _algaeRemoverConfig;
    SparkMaxConfig _algaeIntakeConfig;
    SparkMaxConfig _algaeIntakeArmConfig;

    _algaeRemoverConfig.absoluteEncoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _algaeIntakeConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
    _algaeIntakeArmConfig.absoluteEncoder.PositionConversionFactor(1).VelocityConversionFactor(1);

    _algaeRemoverConfig.SmartCurrentLimit(50);
    _algaeIntakeConfig.SmartCurrentLimit(50);
    _algaeIntakeArmConfig.SmartCurrentLimit(50);


    _algaeRemoverConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.01)
      .I(0)
      .D(0)
      .OutputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .P(0.0001, ClosedLoopSlot::kSlot1)
      .I(0, ClosedLoopSlot::kSlot1)
      .D(0, ClosedLoopSlot::kSlot1)
      .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
      .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);


    _algaeIntakeConfig.closedLoop
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

    _algaeIntakeArmConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.01)
      .I(0)
      .D(0)
      .OutputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .P(0.0001, ClosedLoopSlot::kSlot1)
      .I(0, ClosedLoopSlot::kSlot1)
      .D(0, ClosedLoopSlot::kSlot1)
      .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
      .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);

    _algaeRemover.Configure(_algaeRemoverConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _algaeIntake.Configure(_algaeIntakeConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    _algaeIntakeArm.Configure(_algaeIntakeArmConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);


}

void AlgaeSubsystem::setAlgaeArm(double setIntakeArm){
     intakeArmTotal = setIntakeArm * setIntakeArm * setIntakeArm;
     if (intakeArmTotal > 40) {
        intakeArmTotal = 40;
     }
    if (intakeArmTotal < 10) {
        intakeArmTotal = 10;
    }
    _algaeIntakeArmClosedLoopController.SetReference(intakeArmTotal, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
}

void AlgaeSubsystem::setAlgaeIntakeMotors(double algaeIntakeSpeed){
    _algaeIntake.Set(algaeIntakeSpeed);
}

void AlgaeSubsystem::setRemoverArm(double setFlipperArm){
     removerArmTotal = setFlipperArm * setFlipperArm * setFlipperArm;
    if (removerArmTotal > 40) {
        removerArmTotal = 40;
    }
    if (removerArmTotal < 7) {
        removerArmTotal = 7;
    }
    _algaeRemoverClosedLoopController.SetReference(removerArmTotal, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
}
