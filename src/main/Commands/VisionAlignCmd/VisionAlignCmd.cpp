// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionAlignCmd.h"
#include <frc/Timer.h>

VisionAlignCmd::VisionAlignCmd(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver, bool shouldEnd) : headingSpeedsHelper(headingController, chassis), multiSupplier(std::move(multiSupplier)) {
	this->shooter = shooter;
	this->hood = hood;
	this->chassis = chassis;
	this->launchModeManager = launchModeManager;
	this->driver = driver;
	this->shouldEnd = shouldEnd;

	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ shooter, hood }); //Intake is crashing
}

// Called when the command is initially scheduled.
void VisionAlignCmd::Initialize() {
	chassis->enableSpeedHelper(&headingSpeedsHelper);
}

// Called repeatedly when this Command is scheduled to run
void VisionAlignCmd::Execute() {
	auto launchMode = launchModeManager->getLaunchMode();
	const frc::Pose2d& chassisPose = chassis->getEstimatedPose();\
		bool redAlliance = isRedAlliance();
	frc::Translation2d targetCoords;

	if (launchMode == LaunchModes::Pass) {
		targetCoords = passTargetSwitcher.GetPassTarget(chassisPose, redAlliance);
	} else {
		targetCoords = LaunchConstants::HubPose;
	}
	if (redAlliance) {
		targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
	}


	targetWhileMoving.setTargetLocation(targetCoords);
	frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassisPose.Rotation());
	ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassisPose.Rotation());
	frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassisPose, speed, accel);

	frc::Rotation2d targetAngle((chassisPose.X() - movingGoalLocation.X()).value(), (chassisPose.Y() - movingGoalLocation.Y()).value());
	headingSpeedsHelper.setTargetAngle(targetAngle);


	units::meter_t distanceToTarget = chassisPose.Translation().Distance(movingGoalLocation);
	frc::SmartDashboard::PutNumber("LaunchCommand/DistanceTarget", distanceToTarget.value());

	units::degree_t hoodAngle;
	units::turns_per_second_t shooterSpeed;

	if (launchMode == LaunchModes::Pass) {
		hoodAngle = LaunchConstants::DistanceToHoodForPass[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForPass[distanceToTarget];
	} else {
		hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];
	}


	//Manual
	if (!driver->GetHID().GetAButton()) {
		hood->setHoodAngle(hoodAngle);
		shooter->setObjectiveVelocity(shooterSpeed * multiSupplier());

	} else {
		distanceToTarget = 3.07_m;
		hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];

		hood->setHoodAngle(hoodAngle);
		shooter->setObjectiveVelocity(shooterSpeed * multiSupplier());

	}


	chassisError = units::math::abs((targetAngle - chassisPose.Rotation()).Degrees());
	frc::SmartDashboard::PutNumber("LaunchCommand/ChassisError", chassisError.value());
	frc::SmartDashboard::PutBoolean("LaunchCommand/ChassisErrorBool", chassisError.value() < 3.25);

	frc::SmartDashboard::PutNumber("LaunchCmd", multiSupplier());
	targetPublisher.Set(movingGoalLocation);
}

// Called once the command ends or is interrupted.
void VisionAlignCmd::End(bool interrupted) {
	chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool VisionAlignCmd::IsFinished() {
	if (!shouldEnd) {
		return false;
	}

	return hood->isHoodAtAngle() && chassisError < 3.25_deg;
}