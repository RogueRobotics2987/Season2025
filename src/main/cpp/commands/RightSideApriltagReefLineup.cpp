// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/RightSideApriltagReefLineup.h"

#include <iostream>

RightSideApriltagReefLineup::RightSideApriltagReefLineup(){}
RightSideApriltagReefLineup::RightSideApriltagReefLineup(
  subsystems::CommandSwerveDrivetrain &driveTrain, double setPointX, double setPointY, double setPointYaw)
{
  _driveTrain = &driveTrain;
  AddRequirements({_driveTrain});
  _setPointX = setPointX;
  _setPointY = setPointY;
  _setPointYaw = setPointYaw;

// //maple handles lose tracking for 100 ms sends the same thing
 auto table = nt::NetworkTableInstance::GetDefault().GetTable("MAPLE"); //might cause loop overrun problems!!!
 apriltags_idSub = table->GetDoubleArrayTopic("robot_relative_tagid").Subscribe({});// Getting apriltag data from the network table
 apriltags_xSub = table->GetDoubleArrayTopic("robot_relative_x").Subscribe({});
 apriltags_ySub = table->GetDoubleArrayTopic("robot_relative_y").Subscribe({});
 apriltags_yawSub = table->GetDoubleArrayTopic("robot_relative_yaw").Subscribe({});
}

// Called when the command is initially scheduled.
void RightSideApriltagReefLineup::Initialize() 
{
  std::cout << "this command is being run" << std::endl;
  //change lights
  //make sure robot is robot centric
  finished = false;
}

// Called repeatedly when this Command is scheduled to run
void RightSideApriltagReefLineup::Execute() 
{
  // std::cout << "this command is being run" << std::endl;

  // time++;
  // if(time > 200)
  // {
  //   finished = true;
  //   time = 0;
  // }
  frc::SmartDashboard::PutBoolean("AutoLineup", true);
  // TODO: get list of tags from maple
  // Table: maple
  // Topic: tags: vector<vector<double>>  --- 4 numbers {tag_id, x, y, z, yaw}
  //         an exampe will look like {{3, 0, 1.0, 0.0, 0.8}, {4, 0, 0.5, 0.0, 0.2}, {}, {} }


  // Table: MAPLE
  // Topic: apriltags
  //    type: vector<vector<double>>

  // is this done? 
 std::cout << "getting data" << std::endl;
 std::vector<double> apriltags_id = apriltags_idSub.Get();// Putting apriltag data into vectors
 std::vector<double> apriltags_x = apriltags_xSub.Get();
 std::vector<double> apriltags_y = apriltags_ySub.Get();
 std::vector<double> apriltags_yaw = apriltags_yawSub.Get();

 if(apriltags_id.empty())
 {
   finished = true;

   return;
 }

// std::vector<std::vector<double>> mapleTags{};// Creating the vector in a vector that will hold all the apriltag data

  std::vector<std::vector<double>> mapleTags;
  std::vector<std::vector<double>> allowedMapleTags{};
  std::vector<double> closestAprilTag{-1.0, 0.0, 0.0, 0.0, 0.0};

  std::cout << apriltags_id.size() << std::endl;
  for (int i=  0; i < apriltags_id.size(); i++) 
  { // Repeats for how many numbers are in the apriltag_id vector
    std::cout << "id: " << apriltags_id[i] << std::endl;
    std::cout << "x: " << apriltags_x[i] << std::endl;
    std::cout << "y: " << apriltags_y[i] << std::endl;
    std::cout << "yaw: " << apriltags_yaw[i] << std::endl;
    double cur_id = apriltags_id[i];// Gets the id/x/y/yaw based on which repeat it is on
    double cur_x = apriltags_x[i];
    double cur_y = apriltags_y[i];
    double cur_yaw = apriltags_yaw[i];

    mapleTags.emplace_back(std::vector<double>{cur_id, cur_x, cur_y, cur_yaw});// Puts all the apriltag data into one vector in a vector
  };

  int minDistance = 99999;


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
    if (allowedTag)
    {
      //add to new vector 
      allowedMapleTags.emplace_back(currentTag);
      allowedTag = false;
    }
  }

  //another network table getting our pose??
  std::cout << "idk" << std::endl;
  for(std::vector<double> currentTag: allowedMapleTags)
  {
    std::cout << "ct1: " << currentTag[1] << ", ct2: " << currentTag[2] << std::endl;
    double distance = sqrt(currentTag[1] * currentTag[1] + currentTag[2] * currentTag[2]); // square roots of a^2 + b^2 making it a + b = c
    if(distance < minDistance)
    {
      minDistance = distance;
      closestAprilTag = currentTag;
    }
  }

  frc::SmartDashboard::PutNumber("Tag_ID", closestAprilTag[0]);
  frc::SmartDashboard::PutNumber("Tag_X", closestAprilTag[1]);
  frc::SmartDashboard::PutNumber("Tag_Y", closestAprilTag[2]);
  frc::SmartDashboard::PutNumber("Tag_Yaw", closestAprilTag[3]);


  double currentX = closestAprilTag[1]; // i dont think its supposed to be minDistanceTagID
  double currentY = closestAprilTag[2];
  double currentYaw = closestAprilTag[3]; //placeholder, dont know the how to get the value yet
  double outputX = 0;
  double outputY = 0;
  double outputYaw = 0;

  // these are set points
  errorX = currentX - _setPointX; //tune for the offset of the tag //left side //-0.2
  errorY = currentY - _setPointY; //0.35
  errorYaw = currentYaw - _setPointYaw; // our yaw will always be parallel with the tag //0

  if(errorYaw > 180)
  {
    errorYaw = errorYaw + -360;
  }
  else if(errorYaw < -180)
  {
    errorYaw = errorYaw + 360;
  }
  else
  {
    errorYaw = errorYaw;
  }

  outputX = -errorX * kP_x; //turn the kp positive? ^
  outputY = errorY * kP_y;
  outputYaw = errorYaw * - kP_yaw;

  if(std::fabs(errorYaw) > 20)
  {
    outputX = 0;
    outputY = 0;
  }

 if(outputYaw < 0)
 {
  outputYaw = outputYaw - 6;
 }else if(outputYaw > 0)
 {
  outputYaw = outputYaw + 6;
 }
  //outputX = 0;
  //outputYaw = 0;

  frc::SmartDashboard::PutNumber("output_x", outputX);
  frc::SmartDashboard::PutNumber("outputYaw", outputYaw);

  _driveTrain->SetControl(robotCentricDrive.WithVelocityY(units::meters_per_second_t{outputX})
        .WithVelocityX(units::meters_per_second_t{outputY})
        // .WithVelocityX(units::meters_per_second_t{- _driveStick->GetLeftY()})
        .WithRotationalRate(units::degrees_per_second_t{outputYaw})
   );


  frc::SmartDashboard::PutNumber("error_x", errorX);
  frc::SmartDashboard::PutNumber("error_y", errorY);
  frc::SmartDashboard::PutNumber("error_Yaw", errorYaw);

  //TODO: 
  // now that we have all our apriltags:
  // lock x and fill in with PID
  // lock yaw and fill in with PID
  // allow free y movement

  // TODO: right side only for right now
  //errorx = currentx - desiredx;
  //erroryaw = currentyaw - desiredyaw;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 std::cout << "end of execute" << std::endl;
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
  //return false;
  // if(finished)
  // {
  //   std::cout << "Done No Tag" << std::endl;
  //   return true;
  // }

  if(std::fabs(errorX) < 0.03 && std::fabs(errorY) < 0.03 && std::fabs(errorYaw) < 1.0) //within 5 cm //make another one for the yaw and case if the tag is lost for auto to make sure itll still run
  {
   //change lights
  //  std::cout << "Done!" << std::endl << "\n";
  //  std::cout << errorX << std::endl << "\n";
  //  std::cout << errorYaw << std::endl << "\n";
   return true; //end the command
  }
  else
  {
   std::cout << "Nope!" << std::endl;
   return false;
  }
}