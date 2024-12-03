// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//declare your class

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PController
    : public frc2::CommandHelper<frc2::Command, PController> {
 public:
 //constructor
  PController(double in_kP);

  // Returntype: void, name: Initialize, no parameters (nothing in parenthesis)
  void Initialize() override;

  void Execute() override;

  double kP;

  double Calculate(double current, double setpoint);

  void End(bool interrupted) override;

  // Return typed: bool, name: Isfinished, no parameters
  bool IsFinished() override;
};
