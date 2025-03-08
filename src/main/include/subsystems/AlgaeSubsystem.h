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
     void setAlgaeIntakeMotors(double algaeIntakeSpeed); //intake motors for intake arm set speed
     void setAlgaeArm(double setAlgaeArm);
     void setRemoverArm(double setRemoverArm);
     
    double intakeArmTotal;
    double setIntakeArm;
    double removerArmTotal;
    double setFlipperArm;
    
    // algae arm
    SparkMax _algaeIntakeArm{algaeSubsystemConstants::CANIDAlgaeIntakeArm, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _algaeIntakeArmClosedLoopController = _algaeIntakeArm.GetClosedLoopController();
    SparkAbsoluteEncoder _algaeIntakeArmEncoder = _algaeIntakeArm.GetAbsoluteEncoder();

    // intake
    SparkMax _algaeIntake{algaeSubsystemConstants::CANIDAlgaeIntake, SparkMax::MotorType::kBrushless};
    SparkRelativeEncoder _algaeIntakeEncoder = _algaeIntake.GetEncoder();

    // algae remover
    SparkMax _algaeRemover{algaeSubsystemConstants::CANIDAlgaeRemover, SparkMax::MotorType::kBrushless};
    SparkClosedLoopController _algaeRemoverClosedLoopController = _algaeRemover.GetClosedLoopController();
    SparkAbsoluteEncoder _algaeRemoverEncoder = _algaeRemover.GetAbsoluteEncoder();
    

};