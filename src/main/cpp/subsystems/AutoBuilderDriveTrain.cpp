
#include "subsystems/AutoBuilderDriveTrain.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <iostream>

using namespace pathplanner;
using namespace subsystems;
// using namespace CommandSwerveDrivetrain;

class AutoBuilderDriveTrain : public CommandSwerveDrivetrain {

    public:

        AutoBuilderDriveTrain() {         
            
            AutoBuilder::configure(
                [this](){ return GetState/*getPose*/(); }, // Robot pose supplier
                [this](frc::Pose2d pose){ ResetPose(pose)/*add someothing equalivent to > resetPose(pose)*/; }, // Method to reset odometry (will be called if your auto has a starting pose)
                [this](){ return GetKinematics() /*getRobotRelativeSpeeds()*/; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                [this](auto speedsx, auto speedsy, auto speedsrotational, auto feedforwards){ drivetrain.ApplyRequest([this]() -> auto&& {
            drive.WithVelocityX(speedsx) // Drive forward with negative Y (forward)
                .WithVelocityY(speedsy) // Drive left with negative X (left)
                .WithRotationalRate(speedsrotational); // Drive counterclockwise with negative X (left)
        }) ; /*not the right function */}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
         ),
        //config, // The robot configuration ***config twice so might need to check
        []() {;
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
        }

    private:

};

