// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>
#include "TeleopCurve.h"
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {

 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_leftLeader{0};
  ctre::phoenix6::controls::DutyCycleOut m_leftOut{0};
  frc::XboxController joystick{0};
  TeleopCurve Curve;
   // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;
};
