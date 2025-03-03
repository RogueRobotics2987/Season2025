// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc2/command/CommandPtr.h>

using namespace rev::spark;
using namespace algaeSubsystemConstants;

class AlgaeSubsystem : public frc2::SubsystemBase {
    public:
    AlgaeSubsystem();

    //stuff we use
     void setIntakeMotors(double algaeIntakeSpeed); //intake motors for intake arm set speed
    
    // algae arm
    SparkMax _algaeIntakeArm{CoralSubsystemConstants::CANIDAlgaeIntake, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _elevatorFollowerFirstStageClosedLoopController = _elevatorFollowerFirstStage.GetClosedLoopController();
    SparkRelativeEncoder _elevatorFollowerFirstStageEncoder = _elevatorFollowerFirstStage.GetEncoder();

    // intakeLeft
    SparkMax _algaeIntake{CoralSubsystemConstants::CANIdLeftIntake, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _intakeLeftclosedLoopController = _intakeLeft.GetClosedLoopController(); // TODO: no close loop controllers
    SparkRelativeEncoder _intakeLeftencoder = _intakeLeft.GetEncoder();

    // intakeRight
    SparkMax _algaeRemoverArm{CoralSubsystemConstants::CANIdRightIntake, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _intakeRightclosedLoopController = _intakeRight.GetClosedLoopController();
    SparkRelativeEncoder _intakeRightencoder = _intakeRight.GetEncoder();
    

};