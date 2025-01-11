// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>
#include "TeleopCurve.h"
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>

#include "RobotContainer.h"

using namespace rev::spark;

class Robot : public frc::TimedRobot {
 private:
  SparkMax m_leftLeader{9, SparkMax::MotorType::kBrushless};
  frc::XboxController joystick{0};
  TeleopCurve Curve;
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
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;
};
