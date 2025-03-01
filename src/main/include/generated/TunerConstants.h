#pragma once

#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"

using namespace ctre::phoenix6;

namespace subsystems {
    /* Forward declaration */
    class CommandSwerveDrivetrain;
}

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants::SteerMotorClosedLoopOutput
    static constexpr configs::Slot0Configs steerGains = configs::Slot0Configs{}
        .WithKP(100).WithKI(0).WithKD(0.5)
        .WithKS(0.1).WithKV(2.66).WithKA(0)
        .WithStaticFeedforwardSign(signals::StaticFeedforwardSignValue::UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants::DriveMotorClosedLoopOutput
    static constexpr configs::Slot0Configs driveGains = configs::Slot0Configs{}
        .WithKP(0.1).WithKI(0).WithKD(0)
        .WithKS(0).WithKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    static constexpr swerve::ClosedLoopOutputType kSteerClosedLoopOutput = swerve::ClosedLoopOutputType::Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    static constexpr swerve::ClosedLoopOutputType kDriveClosedLoopOutput = swerve::ClosedLoopOutputType::Voltage;

    // The type of motor used for the drive motor
    static constexpr swerve::DriveMotorArrangement kDriveMotorType = swerve::DriveMotorArrangement::TalonFX_Integrated;
    // The type of motor used for the drive motor
    static constexpr swerve::SteerMotorArrangement kSteerMotorType = swerve::SteerMotorArrangement::TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    static constexpr swerve::SteerFeedbackType kSteerFeedbackType = swerve::SteerFeedbackType::FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    static constexpr units::ampere_t kSlipCurrent = 120_A;

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `With*InitialConfigs()` API documentation.
    static constexpr configs::TalonFXConfiguration driveInitialConfigs{};
    static constexpr configs::TalonFXConfiguration steerInitialConfigs = configs::TalonFXConfiguration{}
        .WithCurrentLimits(
            configs::CurrentLimitsConfigs{}
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .WithStatorCurrentLimit(60_A)
                .WithStatorCurrentLimitEnable(true)
        );

    static constexpr configs::CANcoderConfiguration encoderInitialConfigs{};
    // Configs for the Pigeon 2; leave this nullopt to skip applying Pigeon 2 configs
    static constexpr std::optional<configs::Pigeon2Configuration> pigeonConfigs = std::nullopt;

    static constexpr std::string_view kCANBusName = "canivore";

public:
    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    static inline const CANBus kCANBus{kCANBusName, "./logs/example.hoot"};

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    static constexpr units::meters_per_second_t kSpeedAt12Volts = 5.21_mps;

private:
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    static constexpr units::scalar_t kCoupleRatio = 3.5714285714285716;

    static constexpr units::scalar_t kDriveGearRatio = 6.122448979591837;
    static constexpr units::scalar_t kSteerGearRatio = 21.428571428571427;
    static constexpr units::inch_t kWheelRadius = 2_in;

    static constexpr bool kInvertLeftSide = false;
    static constexpr bool kInvertRightSide = true;

    static constexpr int kPigeonId = 0;

    // These are only used for simulation
    static constexpr units::kilogram_square_meter_t kSteerInertia = 0.01_kg_sq_m;
    static constexpr units::kilogram_square_meter_t kDriveInertia = 0.01_kg_sq_m;
    // Simulated voltage necessary to overcome friction
    static constexpr units::volt_t kSteerFrictionVoltage = 0.2_V;
    static constexpr units::volt_t kDriveFrictionVoltage = 0.2_V;

public:
    static constexpr swerve::SwerveDrivetrainConstants DrivetrainConstants = swerve::SwerveDrivetrainConstants{}
            .WithCANBusName(kCANBusName)
            .WithPigeon2Id(kPigeonId)
            .WithPigeon2Configs(pigeonConfigs);

private:
    static constexpr swerve::SwerveModuleConstantsFactory ConstantCreator =
        swerve::SwerveModuleConstantsFactory<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>{}
            .WithDriveMotorGearRatio(kDriveGearRatio)
            .WithSteerMotorGearRatio(kSteerGearRatio)
            .WithCouplingGearRatio(kCoupleRatio)
            .WithWheelRadius(kWheelRadius)
            .WithSteerMotorGains(steerGains)
            .WithDriveMotorGains(driveGains)
            .WithSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .WithDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .WithSlipCurrent(kSlipCurrent)
            .WithSpeedAt12Volts(kSpeedAt12Volts)
            .WithDriveMotorType(kDriveMotorType)
            .WithSteerMotorType(kSteerMotorType)
            .WithFeedbackSource(kSteerFeedbackType)
            .WithDriveMotorInitialConfigs(driveInitialConfigs)
            .WithSteerMotorInitialConfigs(steerInitialConfigs)
            .WithEncoderInitialConfigs(encoderInitialConfigs)
            .WithSteerInertia(kSteerInertia)
            .WithDriveInertia(kDriveInertia)
            .WithSteerFrictionVoltage(kSteerFrictionVoltage)
            .WithDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    static constexpr int kFrontLeftDriveMotorId = 3;
    static constexpr int kFrontLeftSteerMotorId = 7;
    static constexpr int kFrontLeftEncoderId = 11;
    static constexpr units::turn_t kFrontLeftEncoderOffset = 0.063720703125_tr;
    static constexpr bool kFrontLeftSteerMotorInverted = true;
    static constexpr bool kFrontLeftEncoderInverted = false;

    static constexpr units::inch_t kFrontLeftXPos = 12.25_in;
    static constexpr units::inch_t kFrontLeftYPos = 12.25_in;

    // Front Right
    static constexpr int kFrontRightDriveMotorId = 4;
    static constexpr int kFrontRightSteerMotorId = 8;
    static constexpr int kFrontRightEncoderId = 12;
    static constexpr units::turn_t kFrontRightEncoderOffset = 0.42724609375_tr;
    static constexpr bool kFrontRightSteerMotorInverted = true;
    static constexpr bool kFrontRightEncoderInverted = false;

    static constexpr units::inch_t kFrontRightXPos = 12.25_in;
    static constexpr units::inch_t kFrontRightYPos = -12.25_in;

    // Back Left
    static constexpr int kBackLeftDriveMotorId = 2;
    static constexpr int kBackLeftSteerMotorId = 6;
    static constexpr int kBackLeftEncoderId = 10;
    static constexpr units::turn_t kBackLeftEncoderOffset = 0.19482421875_tr;
    static constexpr bool kBackLeftSteerMotorInverted = true;
    static constexpr bool kBackLeftEncoderInverted = false;

    static constexpr units::inch_t kBackLeftXPos = -12.25_in;
    static constexpr units::inch_t kBackLeftYPos = 12.25_in;

    // Back Right
    static constexpr int kBackRightDriveMotorId = 1;
    static constexpr int kBackRightSteerMotorId = 5;
    static constexpr int kBackRightEncoderId = 9;
    static constexpr units::turn_t kBackRightEncoderOffset = -0.141845703125_tr;
    static constexpr bool kBackRightSteerMotorInverted = true;
    static constexpr bool kBackRightEncoderInverted = false;

    static constexpr units::inch_t kBackRightXPos = -12.25_in;
    static constexpr units::inch_t kBackRightYPos = -12.25_in;


public:
    static constexpr swerve::SwerveModuleConstants FrontLeft = ConstantCreator.CreateModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted);
    static constexpr swerve::SwerveModuleConstants FrontRight = ConstantCreator.CreateModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted);
    static constexpr swerve::SwerveModuleConstants BackLeft = ConstantCreator.CreateModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted);
    static constexpr swerve::SwerveModuleConstants BackRight = ConstantCreator.CreateModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted);

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program.
     */
    static subsystems::CommandSwerveDrivetrain CreateDrivetrain();
};


/**
 * \brief Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
 */
class TunerSwerveDrivetrain : public swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder> {
public:
    using SwerveModuleConstants = swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>;

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * \param modules             Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    TunerSwerveDrivetrain(swerve::SwerveDrivetrainConstants const &driveTrainConstants, ModuleConstants const &... modules) :
        SwerveDrivetrain{driveTrainConstants, modules...}
    {}

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    TunerSwerveDrivetrain(
        swerve::SwerveDrivetrainConstants const &driveTrainConstants,
        units::hertz_t odometryUpdateFrequency,
        ModuleConstants const &... modules
    ) :
        SwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency, modules...}
    {}

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                   in the form [x, y, theta]ᵀ, with units in meters
     *                                   and radians
     * \param visionStandardDeviation    The standard deviation for vision calculation
     *                                   in the form [x, y, theta]ᵀ, with units in meters
     *                                   and radians
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    TunerSwerveDrivetrain(
        swerve::SwerveDrivetrainConstants const &driveTrainConstants,
        units::hertz_t odometryUpdateFrequency,
        std::array<double, 3> const &odometryStandardDeviation,
        std::array<double, 3> const &visionStandardDeviation,
        ModuleConstants const &... modules
    ) :
        SwerveDrivetrain{
            driveTrainConstants, odometryUpdateFrequency,
            odometryStandardDeviation, visionStandardDeviation, modules...
        }
    {}
};
