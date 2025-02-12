// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/CommandSwerveDrivetrain.h"
//#include lights: have brandon change the name!!!


#include "Telemetry.h"
#include "RobotContainer.h"
#include "commands/ApriltagReefLineup.h"

ApriltagReefLineup::ApriltagReefLineup() {}
ApriltagReefLineup::ApriltagReefLineup(Telemetry &drivePose, RobotContainer &drivetrain) { //needs xbox perm?
  m_drive = &drivetrain;
  m_drivePose = &drivePose;
  //AddRequirements({m_drivetrain});  


}
// Called when the command is initially scheduled.
void ApriltagReefLineup::Initialize() {
  state = 0; // dont think we need
  time = 0;
  finished = false;

  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0); // change limelight to geting the maple values?

}

// Called repeatedly when this Command is scheduled to run
void ApriltagReefLineup::Execute() {
  frc::SmartDashboard::PutBoolean("AutoLineup", true);

  // if(state == 0) // state stuff is from note but body isnt, dont think you need
  //{
    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tv", 0) > 0) // this needs to be changed
    {
      hasSeen = true;
      direction = -1; 
      //m_lights->GBChase(); //replace m_lights
    }
    else
    {
      direction = 1;
      //m_lights->loop(); //replace m_lights
    }

    //currentHeading = m_drive->GetPose().Rotation().Degrees().value(); // change m_drive //in AutoAprilTag so might need to be somewhere else

    //rotApril = units::angular_velocity::radians_per_second_t(m_limelight->GetApriltagDriveMotorVal(currentHeading, lastHeading)); // change limelight
   
    speedY = Deadzone(m_driverController->GetLeftY());
    speedX = Deadzone(m_driverController->GetLeftX());
    rot = Deadzone(m_driverController->GetRightX());

    if((fabs(speedY) + fabs(speedX) + fabs(rot)) < .05) //re-look at // xpose 
    {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
    }

    if(hasSeen == true)
    {
      if(fabs(rotApril.value()) > 0.05)
      {
        //m_drive->Drive(units::velocity::meters_per_second_t(speedY * 4), units::velocity::meters_per_second_t(speedX * 4), rotApril, false, false); // change m_drive and maybe perm
      }
      else
      {
        //m_drive->Drive(units::velocity::meters_per_second_t(speedY * 4), units::velocity::meters_per_second_t(speedX * 4), rotApril, false, NoJoystickInput); // change m_drive and maybe perm //why different from above?
      }
    }
    else
    {
      // m_drive->Drive(
      //   units::velocity::meters_per_second_t(speedY * 4),
      //   units::velocity::meters_per_second_t(speedX * 4),
      //   units::radians_per_second_t(-rot * AutoConstants::kMaxAngularSpeed),
      //   false,
      //   NoJoystickInput
      // );
    }

    // if(m_limelight->GetNumTargets() > 0) // change m_limelight //move?
    // {
    //   hasSeen = true;
    // }

    //updating the last heading
    lastHeading = currentHeading;
}


//}
// Called once the command ends or is interrupted.
void ApriltagReefLineup::End(bool interrupted) {
  frc::SmartDashboard::PutBoolean("AutoLineup", false);

  // if(finished == true)
  // {
  //   m_lights->SetNoColor(); // change m_lights
  // } 
  // else 
  // {  
  //   m_lights->SetLightsGreen(); // change m_lights
  // }
}

// Returns true when the command should end.
bool ApriltagReefLineup::IsFinished() {
  return false;
}

float ApriltagReefLineup::Deadzone(float x) // do we need deadzone in this? what should it be
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