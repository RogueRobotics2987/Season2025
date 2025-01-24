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

// 1. comment out autobuilder stuff to get code to compile
// 2. add cout to copy constructor below to make sure this constructor is actually called in the code. build and deploy.
// 3. once you see the cout in the log window, then add the path planner autobuilder stuff here
//CommandSwerveDrivetrain::CommandSwerveDrivetrain(const CommandSwerveDrivetrain& other_object) {

    //std::cout << "the copy constructor is not being overridden!";

//}

void CommandSwerveDrivetrain::ConfigureAutoBuilder(){
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        [this] {return GetState().Pose; },
        [this] (frc::Pose2d const &pose) {return ResetPose(pose); },
        [this] {return GetState().Speeds; },
        [this](frc::ChassisSpeeds const &speeds, pathplanner::DriveFeedforwards const &feedforwards) {
            return SetControl(
                m_pathApplyRobotSpeeds.WithSpeeds(speeds)
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
            );
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // PID constants for translation
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            // PID constants for rotation
            pathplanner::PIDConstants{7.0, 0.0, 0.0}
        ),
        std::move(config),
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        [] {
            auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
            return alliance == frc::DriverStation::Alliance::kRed;
        },
        this // Subsystem for requirements
    );
}

void CommandSwerveDrivetrain::Periodic()
{
    RobotConfig config = RobotConfig::fromGUISettings();
    
    // AutoBuilder::configure(
    // [this](){ return GetState/*getPose*/(); }, // Robot pose supplier
    // [this](frc::Pose2d pose){ ResetPose(pose)/*add someothing equalivent to > resetPose(pose)*/; }, // Method to reset odometry (will be called if your auto has a starting pose)
    // [this](){ return GetKinematics() /*getRobotRelativeSpeeds()*/; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    // [this](auto speeds, auto feedforwards){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    // std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //     PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //     PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //   ),
    //     config, // The robot configuration ***config twice so might need to check
    //     []() {
    //         // Boolean supplier that controls when the path will be mirrored for the red alliance
    //         // This will flip the path being followed to the red side of the field.
    //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //         auto alliance = frc::DriverStation::GetAlliance();
    //         if (alliance) {
    //             return alliance.value() == frc::DriverStation::Alliance::kRed;
    //         }
    //         return false;
    //     },
    //     this // Reference to this subsystem to set requirements
    // );
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}
