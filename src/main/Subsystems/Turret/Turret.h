// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "TurretConstants.h"
#include "Constants.h"
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <units/math.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"

class Turret : public frc2::SubsystemBase {
public:
	Turret(Chassis* chassis);

	void setTargetAngle(units::degree_t turretTarget);

	frc::Rotation2d GetTurretAimingParameterFromRobotPose(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition);

	void AimAtFieldPosition(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition);

	bool isAimAtFieldPosition(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition);

	frc::Pose2d GetTurretPose(const frc::Pose2d& robotPose);
	const frc::Transform3d& GetRobotToTurret();

	frc2::CommandPtr TestCommand(units::degree_t setPoint);

	double getForceFactorCables(units::degree_t turretAngleDegrees);

	const units::degree_t& GetRobotRelativeHeading();

	bool isMotorAtPosition();

	void UpdateTelemetry();

	void Periodic() override;

private:

	units::degree_t convertToClosestBoundedTurretAngleDegrees(units::degree_t targetAngleDegrees);
	units::degree_t calculateTurretAngleFromCANCoderDegrees();

	bool enableChassisComp = false;
	frc::Transform3d robotToTurret;

	units::degree_t turretActualAngle;

	OverTalonFX turretMotor{ TurretConstants::TurretConfig(), robotConstants::rio };
	OverCANCoder turret1CANCoder{ TurretConstants::Turret2CANConfig(), robotConstants::rio };
	OverCANCoder turret2CANCoder{ TurretConstants::Turret1CANConfig(), robotConstants::rio };

	nt::StructPublisher<frc::Pose2d> turretPublisher =
		nt::NetworkTableInstance::GetDefault().GetStructTopic < frc::Pose2d
		>("SmartDashboard/TurretPose").Publish();

	nt::StructPublisher<frc::Pose3d> cameraTurretPublisher =
		nt::NetworkTableInstance::GetDefault().GetStructTopic < frc::Pose3d
		>("SmartDashboard/CameraTurretPose").Publish();


	// ctre::phoenix6::controls::VoltageOut turretVoltageRequest{0.0_V};
	ctre::phoenix6::controls::PositionVoltage turretVoltageRequest{ 0.0_tr };


	// frc::ProfiledPIDController<units::degree> turretPID {0.1, 0.0, 0.0, {TurretConstants::TurretVelocity,
	//         TurretConstants::TurretAcceleration}, RobotConstants::LoopTime};

	Chassis* chassis;

};
