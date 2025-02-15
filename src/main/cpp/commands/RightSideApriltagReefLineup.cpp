// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/RightSideApriltagReefLineup.h"

RightSideApriltagReefLineup::RightSideApriltagReefLineup() {}
RightSideApriltagReefLineup::RightSideApriltagReefLineup(subsystems::CommandSwerveDrivetrain &driveTrain, RobotContainer &robotContainer)
{ // i dont think we need the drivepose object
  _driveTrain = &driveTrain;
  _robotContainer = &robotContainer;
  AddRequirements({&driveTrain});
}
// Called when the command is initially scheduled.
void RightSideApriltagReefLineup::Initialize() 
{
  //change lights
  hasSeen = false;
  finished = false;
  //updating the last heading
  //lastHeading = currentHeading; //do we need??
  
  nt::NetworkTableInstance::GetDefault().GetTable("maple"); // whats the name of the maple table? //maple

}

// Called repeatedly when this Command is scheduled to run
void RightSideApriltagReefLineup::Execute() 
{
  frc::SmartDashboard::PutBoolean("AutoLineup", true);
  // TODO: get list of tags from maple
  // Table: maple
  // Topic: tags: vector<vector<double>>  --- 4 numbers {tag_id, x, y, z, yaw}
  //         an exampe will look like {{3, 0, 1.0, 0.0, 0.8}, {4, 0, 0.5, 0.0, 0.2}, {}} {{}}
  // 

  std::vector<std::vector<double>> mapleTags{{3, 0.3, 0.3, 0.0, 0.5}, {5, 0.6, 0.0, 0.0, 0.3}};
  std::vector<std::vector<double>> allowedMapleTags{};
  std::vector<double> closestAprilTag{-1.0, 0.0, 0.0, 0.0, 0.0};
  
  int minDistance = 99999;
  int currentx = closestAprilTag[1]; // i dont think its supposed to be minDistanceTagID
  double currentyaw = closestAprilTag[5]; //placeholder, dont know the how to get the value yet
  double errorX = currentx - 0.25;
  double erroryaw = currentyaw - 0;
  double outPutx = 0;
  double outPutyaw = 0;
  
  //take networktable and get the apriltags it sees

  for(std::vector<double> currentTag: mapleTags)
  {
    bool allowedTag = false;

    for(int currentReefTagID: _reefTags)
    {
      if(currentTag[0] == currentReefTagID)
      {
        allowedTag = true;
      }
    }
    if(allowedTag == true)
    {
      //add to new vector 
      allowedMapleTags.emplace_back(currentTag);
      //TODO: allowedTag turn back false
    }
  }

  //another network table getting our pose??

  for(std::vector<double> currentTag: allowedMapleTags)
  {
    double distance = sqrt(currentTag[1] * currentTag[1] + currentTag[2] * currentTag[2]); // square roots of a^2 + b^2 making it a + b = c
    if(distance < minDistance)
    {
      minDistance = distance;
      closestAprilTag[0] = currentTag[0];
    }
  }

  _driveTrain->SetControl([this]() -> auto&& {
      return _robotContainer->drive.WithVelocityX(units::meters_per_second_t(5)) // Drive forward with positive Y (forward)
                  .WithVelocityY(units::meters_per_second_t(2)) // Drive left with positive X (left)
                  .WithRotationalRate(units::degrees_per_second_t(1)); // Drive counterclockwise with negative X (left)
  })

  //TODO: 
  // now that we have all our apriltags:
  // lock x and fill in with PID
  // lock yaw and fill in with PID
  // allow free y movement

  // if(currentposex > errorx)
  // {
  //   keep moving our x 
  // }
  // else
  // {
  //   stop moving and wait
  // }

  // if(currentposeyaw > erroryaw)
  // {
  //   keep moving our yaw
  // }
  // else
  // {
  //   stop and wait
  // }

  // TODO: right side only for right now
  //errorx = currentx - desiredx;
  //erroryaw = currentyaw - desiredyaw;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   
}

// Called once the command ends or is interrupted.
void RightSideApriltagReefLineup::End(bool interrupted) 
{
  frc::SmartDashboard::PutBoolean("AutoLineup", false);

  // if(finished == true)
  // {
  //   m_lights->SetNoColor(); // change m_lights
  // } 
  // else 
  // {  
  //   m_lights->SetLightsRed(); // change m_lights
  // }
}

// Returns true when the command should end.
bool RightSideApriltagReefLineup::IsFinished() 
{
  //set all variables to what their start is just in case
  return false;
}

float RightSideApriltagReefLineup::Deadzone(float x) // do we need to mess with deadzone in this? what should it be
{
  if ((x < 0.1) && (x > -0.1))
  {
    x = 0;
  }
  else if (x >= 0.1)
  {
    x = x - 0.1;
  }
  else if (x <= -0.1)
  {
    x = x + 0.1;
  }
  return(x);
}