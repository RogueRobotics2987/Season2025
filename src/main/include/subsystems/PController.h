// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <iostream>
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

  PController(double in_kP);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  double Calculate(double current, double setposition);

  bool IsFinished() override;

 private:

  double kP;
};
