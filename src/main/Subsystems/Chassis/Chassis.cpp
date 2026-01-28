#include "Chassis.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"
#include <units/angular_acceleration.h>
#include <frc/RobotController.h>
#include "Constants.h"

// Initialize static members

frc::SimpleMotorFeedforward<units::meters> feedForwardFrontLeft {0.0_V, 2.0879_V / 1_mps, 0.098433_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardFrontRight {0.0_V, 2.0879_V / 1_mps, 0.098433_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardBackLeft {0.0_V, 2.0879_V / 1_mps, 0.098433_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardBackRight {0.0_V, 2.0879_V / 1_mps, 0.098433_V / 1_mps_sq};

Chassis::Chassis() : SwerveChassis() {
    configureSwerveBase();
    setAcceptingVisionMeasurements(true);
    resetHeading();
}

units::meters_per_second_t Chassis::getMaxModuleSpeed() {
    return 5.12_mps; //5mps
}

units::meter_t Chassis::getDriveBaseRadius() {
    return 0.5270_m;
}

SwerveModule& Chassis::getBackLeftModule() {
    return backLeftModule;
}

SwerveModule& Chassis::getBackRightModule() {
    return backRightModule;
}

SwerveModule& Chassis::getFrontLeftModule() {
    return frontLeftModule;
}

SwerveModule& Chassis::getFrontRightModule() {
    return frontRightModule;
}

frc::SwerveDriveKinematics<4>& Chassis::getKinematics() {
    return kinematics;
}

frc::Rotation2d Chassis::getRotation2d() {
    return pigeon.GetRotation2d();
}

frc::Rotation3d Chassis::getRotation3d() {
    return pigeon.GetRotation3d();
}

SwerveModuleConfig Chassis::FrontLeftConfig() {
    SwerveModuleConfig config {feedForwardFrontLeft};
    config.DriveMotorConfig.MotorId = 2;
    config.TurnMotorConfig.MotorId = 1;
    config.EncoderConfig.CanCoderId = 9;
    config.CanBus = robotConstants::rio;
    config.DriveGearRatio = 6.03;
    config.TurnGearRatio = 287.0 / 11.0;
    config.WheelDiameter = 4_in;
#ifndef __FRC_ROBORIO__
    config.EncoderConfig.Offset = 0.0_tr;
#else
	config.EncoderConfig.Offset = 0.218994140625_tr;
#endif 
    config.TurnMotorConfig.PIDConfigs.WithKP(40).WithKS(0.15);
    config.TurnMotorConfig.Inverted = false;
    config.ModuleName = "Front Left";
    return config;
}

SwerveModuleConfig Chassis::FrontRightConfig() {
    SwerveModuleConfig config {feedForwardFrontRight};
    config.DriveMotorConfig.MotorId = 4;
    config.TurnMotorConfig.MotorId = 3;
    config.EncoderConfig.CanCoderId = 12;
    config.CanBus = robotConstants::rio;
    config.DriveGearRatio = 6.03;
    config.TurnGearRatio = 287.0 / 11.0;
    config.WheelDiameter = 4_in;
#ifndef __FRC_ROBORIO__
    config.EncoderConfig.Offset = 0.0_tr;
#else
	config.EncoderConfig.Offset = 0.105712890625_tr;
#endif 
    config.TurnMotorConfig.PIDConfigs.WithKP(40).WithKS(0.15);
    config.TurnMotorConfig.Inverted = false;
    config.ModuleName = "Front Right";
    return config;
}

SwerveModuleConfig Chassis::BackLeftConfig() {

    SwerveModuleConfig config {feedForwardBackLeft};
    config.DriveMotorConfig.MotorId = 6;
    config.TurnMotorConfig.MotorId = 5;
    config.EncoderConfig.CanCoderId = 10;
    config.CanBus = robotConstants::rio;
    config.DriveGearRatio = 6.03;
    config.TurnGearRatio = 287.0 / 11.0;
    config.WheelDiameter = 4_in;
#ifndef __FRC_ROBORIO__
    config.EncoderConfig.Offset = 0.0_tr;
#else
	config.EncoderConfig.Offset = -0.37109375_tr;
#endif
    config.TurnMotorConfig.PIDConfigs.WithKP(40).WithKS(0.15);
    config.TurnMotorConfig.Inverted = false;
    config.ModuleName = "Back Left";
    return config;
}

SwerveModuleConfig Chassis::BackRightConfig() {
    SwerveModuleConfig config {feedForwardBackRight};
    config.DriveMotorConfig.MotorId = 8;
    config.TurnMotorConfig.MotorId = 7;
    config.EncoderConfig.CanCoderId = 11;
    config.CanBus = robotConstants::rio;
    config.DriveGearRatio = 6.03;
    config.TurnGearRatio = 287.0 / 11.0;
    config.WheelDiameter = 4_in;
#ifndef __FRC_ROBORIO__
    config.EncoderConfig.Offset = 0.0_tr;
#else
	config.EncoderConfig.Offset = 0.3681640625_tr;
#endif 
    config.TurnMotorConfig.PIDConfigs.WithKP(40).WithKS(0.15);
    config.TurnMotorConfig.Inverted = false;
    config.ModuleName = "Back Right";
    return config;
}