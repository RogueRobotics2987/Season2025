// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;

nt::DoubleArraySubscriber positionSub;
nt::DoubleArraySubscriber orientationSub;

Robot::Robot() {}

void Robot::RobotInit() {

    // Add to your RobotInit, Command, or Subsystem
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("MAPLE");
  positionSub = table->GetDoubleArrayTopic("position").Subscribe({});
  orientationSub = table->GetDoubleArrayTopic("orientation").Subscribe({});
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
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
  
    // Get the latest value for position and orientation. If the value hasn't been updated since the last time
   // we read the table, use the same value.
  std::vector<double> position = positionSub.Get();
  std::vector<double> orientation = orientationSub.Get();

  std::cout << "position XYZ: ";
  for (double pos : position) {
    std::cout << pos << " ";
  }
  std::cout << std::endl;

  std::cout << "orientation RPY: ";
  for (double angle : orientation) {
    std::cout << angle << " ";  }
  std::cout << std::endl;
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
