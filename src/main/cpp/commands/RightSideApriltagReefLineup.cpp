// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/RightSideApriltagReefLineup.h"

RightSideApriltagReefLineup::RightSideApriltagReefLineup(subsystems::CommandSwerveDrivetrain &driveTrain, RobotContainer &robotContainer) : 
_driveTrain(driveTrain) 
,_robotContainer(robotContainer)
{
  AddRequirements({&_driveTrain});
}

// Called when the command is initially scheduled.
void RightSideApriltagReefLineup::Initialize() 
{
  //change lights
  hasSeen = false;
  finished = false;
  //make sure robot is robot centric
  
  nt::NetworkTableInstance::GetDefault().GetTable("maple");
}

// Called repeatedly when this Command is scheduled to run
void RightSideApriltagReefLineup::Execute() 
{
  frc::SmartDashboard::PutBoolean("AutoLineup", true);
  // TODO: get list of tags from maple
  // Table: maple
  // Topic: tags: vector<vector<double>>  --- 4 numbers {tag_id, x, y, z, yaw}
  //         an exampe will look like {{3, 0, 1.0, 0.0, 0.8}, {4, 0, 0.5, 0.0, 0.2}, {}, {} }
  // 

  // Table: MAPLE
  // Topic: apriltags
  //    type: vector<vector<double>>

  // is this done? 

nt::DoubleArraySubscriber apriltags_idSub;// Creates the variables that hold the apriltag data
nt::DoubleArraySubscriber apriltags_xSub;
nt::DoubleArraySubscriber apriltags_ySub;
nt::DoubleArraySubscriber apriltags_yawSub;


auto table = nt::NetworkTableInstance::GetDefault().GetTable("MAPLE");
apriltags_idSub = table->GetDoubleArrayTopic("apriltag_id").Subscribe({});// Getting apriltag data from the network table
apriltags_xSub = table->GetDoubleArrayTopic("apriltag_x").Subscribe({});
apriltags_ySub = table->GetDoubleArrayTopic("apriltag_y").Subscribe({});
apriltags_yawSub = table->GetDoubleArrayTopic("apriltag_yaw").Subscribe({});

std::vector<double> apriltags_id = apriltags_idSub.Get();// Putting apriltag data into vectors
std::vector<double> apriltags_x = apriltags_xSub.Get();
std::vector<double> apriltags_y = apriltags_ySub.Get();
std::vector<double> apriltags_yaw = apriltags_yawSub.Get();

std::vector<std::vector<double>> mapleTags{};// Creating the vector in a vector that will hold all the apriltag data

for (int i=0; i>apriltags_id.size(); i++) 
{ // Repeats for how many numbers are in the apriltag_id vector
  double cur_id = apriltags_id[i];// Gets the id/x/y/yaw based on which repeat it is on
  double cur_x = apriltags_x[i];
  double cur_y = apriltags_y[i];
  double cur_yaw = apriltags_yaw[i];

  mapleTags.emplace_back(std::vector<double>{cur_id, cur_x, cur_y, cur_yaw});// Puts all the apriltag data into one vector in a vector
};

  //std::vector<std::vector<double>> mapleTags{{3, 0.3, 0.3, 0.0, 0.5}, {5, 0.6, 0.0, 0.0, 0.3}};  Replaced by the mapleTag data on line 62/70
  std::vector<std::vector<double>> allowedMapleTags{};
  std::vector<double> closestAprilTag{-1.0, 0.0, 0.0, 0.0, 0.0};
  
  int minDistance = 99999;
  int currentx = closestAprilTag[1]; // i dont think its supposed to be minDistanceTagID
  double currentyaw = closestAprilTag[5]; //placeholder, dont know the how to get the value yet
  double outPutx = 0;
  double outPutyaw = 0;
  errorX = currentx - 0.25; //desired x?
  erroryaw = currentyaw - 0; // desired y?
  outPutx = errorX * kP_x;
  outPutyaw = erroryaw * kP_yaw;
  
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

  _driveTrain.SetControl(robotCentricDrive.WithVelocityX(units::meters_per_second_t{kP_x})
        .WithVelocityY(units::meters_per_second_t{2})
        .WithRotationalRate(units::degrees_per_second_t{kP_yaw})
   );

  //TODO: 
  // now that we have all our apriltags:
  // lock x and fill in with PID
  // lock yaw and fill in with PID
  // allow free y movement

  // TODO: right side only for right now
  //errorx = currentx - desiredx;
  //erroryaw = currentyaw - desiredyaw;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   
}

// Called once the command ends or is interrupted.
void RightSideApriltagReefLineup::End(bool interrupted) 
{
  frc::SmartDashboard::PutBoolean("AutoLineup", false);
}

// Returns true when the command should end.
bool RightSideApriltagReefLineup::IsFinished() 
{
  //set all variables to what their start is just in case?

  if(errorX + erroryaw <= 0.5)
  {
    //change lights
    return true; //end the command
  }
  else
  {
    return false;
  }
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