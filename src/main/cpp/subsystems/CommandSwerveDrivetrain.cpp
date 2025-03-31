#include "subsystems/CommandSwerveDrivetrain.h"
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/RobotController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
// #include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace pathplanner;
using namespace subsystems;

void CommandSwerveDrivetrain::MapleInit(){
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("MAPLE");
    /*if (table == nullptr) {
        std::cout << "AHHHHHHHH" << std::endl;
    }else{
        std::cout << "OOOOOH" << std::endl; // this one prints*/
    positionSub = table->GetDoubleArrayTopic("position").Subscribe({});
    orientationSub = table->GetDoubleArrayTopic("orientation").Subscribe({});

    // if (positionSub == nullptr) {
    //     std::cout << "AHHHHHHHH22222" << std::endl;
    // }else{
    //     std::cout << "OOOOOH22222" << std::endl;
    // }
    // orientationSub = table->GetDoubleArrayTopic("orientation").Subscribe({});
}

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

    // Grabs the position and orintation values from the network table 'MAPLE'
    std::vector<double> position = positionSub.Get(); //elements x, y, z
    std::vector<double> orientation = orientationSub.Get(); //elements roll, pitch, yaw
    double xPose = position[0]; // taking x
    double yPose = position[1]; // taking y
    double yawAngle = orientation[2]; // taking yaw
   
    // units::meter_t xPoseInMeters = units::meter_t {xPose}; 
    // units::meter_t yPoseInMeters = units::meter_t {yPose};
    // units::angle::degree_t yawAngleInDegree = units::angle::degree_t {yawAngle};
   
    // frc::Rotation2d yawRotation2D = frc::Rotation2d(yawAngleInDegree);

    // frc::Pose2d pose2D(xPoseInMeters, yPoseInMeters, yawRotation2D);
    
    // units::time::second_t epochStartupTime = utils::GetCurrentTime();   //find utils for currenttime () 
    // units::time::second_t convertedEpochStartupTime = utils::FPGAToCurrentTime(epochStartupTime);
    // AddVisionMeasurement(pose2D, convertedEpochStartupTime); //make pose2d()


    // std::cout << xPose << std::endl;
    // std::cout << yPose << std::endl;
    // std::cout << yawAngle << std::endl;
    //TODO: TEST PRINT OF THE VALUES; Delete these once testing is complete - we don't want to print 6 lines every cycle.
    // std::cout << "position XYZ: ";
    // for (double pos : position) {
    //     std::cout << pos << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "orientation RPY: ";
    // for (double angle : orientation) {
    //     std::cout << angle << " ";
    // }
    // std::cout << std::endl;
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
