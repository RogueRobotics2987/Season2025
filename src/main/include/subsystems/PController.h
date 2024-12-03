// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <iostream>
#include <string>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
// Create your PController class!!
// 1. Do not delete any of the functions that were automatically generated
// 2. Create a new function called Calculate, this will preform the PController calculation.
// 3. Return type : double
// 4. Two parameters, both double: current, setpoint(desired)
// 5. The constructor for the class should tkae in a value for kP and store it in the object.
class PController
    : public frc2::CommandHelper<frc2::Command, PController> {
 public:
    // Constructor
  PController(double in_kP);


  // Retrun type: void, name: Initailize, no parameters (nothing in parenthesis)
  void Initialize() override;

  double Calculate(double current, double setpoint);
  

  void Execute() override;

  void End(bool interrupted) override;
  // Return typed: bool, name: IsFinished  
  bool IsFinished() override;

  private:
    double kP;
};
