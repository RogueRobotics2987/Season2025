// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <rev/config/SparkMaxConfig.h>



Robot::Robot() {
  /*
   * Create a new SPARK MAX configuration object. This will store the
   * configuration parameters for the SPARK MAX that we will set below.
   */
  SparkMaxConfig motorConfig;

  /*
   * Configure the encoder. For this we used the rev through bore encoder. If
   * needed, we can adjust values like the position or velocity conversion
   * factors.
   * Configure the closed loop controller. We want to make sure we set the
   * feedback sensor as the through bore encoder.
   */
   motorConfig.absoluteEncoder.PositionConversionFactor(1).VelocityConversionFactor(1);  
   
   motorConfig.closedLoop
       
/**
* By default, the PID controller will use the Hall sensor from a NEO or NEO 550 for
* its feedback device. Instead, we can set the feedback device to the alternate
* encoder object
*/
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
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

  /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
   */
  m_motor.Configure(motorConfig, SparkBase::ResetMode::kResetSafeParameters,
                    SparkBase::PersistMode::kPersistParameters);

  // Initialize dashboard values
  frc::SmartDashboard::SetDefaultNumber("Target Position", 0);
  frc::SmartDashboard::SetDefaultNumber("Target Velocity", 0);
  frc::SmartDashboard::SetDefaultBoolean("Control Mode", false);
  frc::SmartDashboard::SetDefaultBoolean("Reset Encoder", false);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

   // Display encoder position and velocity
  frc::SmartDashboard::PutNumber("Actual Position", m_throughBoreEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Actual Velocity", m_throughBoreEncoder.GetVelocity());

  /*
  if (frc::SmartDashboard::GetBoolean("Reset Encoder", false)) {
    frc::SmartDashboard::PutBoolean("Reset Encoder", false);
    // Reset the encoder position to 0
    m_encoder.SetPosition(0);
  
  }
  */
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {


  if (frc::SmartDashboard::GetBoolean("Control Mode", false)) {
    
    // Get the target velocity from SmartDashboard and set it as the
    // * setpoint for the closed0 loop controller.
     
    double targetVelocity = frc::SmartDashboard::GetNumber("Target Velocity", 0);
    frc::SmartDashboard::PutNumber("Our Target Velocity", targetVelocity);
    m_closedLoopController.SetReference(targetVelocity, SparkMax::ControlType::kVelocity, ClosedLoopSlot::kSlot1);
  } else {
    
    // * Get the target position from SmartDashboard and set it as the
    // * setpoint for the closed loop controller.
     
    double targetPosition = frc::SmartDashboard::GetNumber("Target Position", 0);
    frc::SmartDashboard::PutNumber("Our Target Position", targetPosition);
    m_closedLoopController.SetReference(targetPosition, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0);
  }
  
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif