// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"

LaunchCommand::LaunchCommand(Shooter* shooter, Chassis* chassis, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver) : multiSupplier(std::move(multiSupplier)) {

	this->shooter = shooter;
	this->chassis = chassis;
	this->launchModeManager = launchModeManager;
	this->driver = driver;

	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ shooter });
}


// Called when the command is initially scheduled.
void LaunchCommand::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void LaunchCommand::Execute() {
	auto sideMode = launchModeManager->getSideMode();
	frc::Translation2d targetCoords;
	if (sideMode == SideMode::Left) {
		targetCoords = LaunchConstants::LeftPass;
	} else if (sideMode == SideMode::Right) {
		targetCoords = LaunchConstants::RightPass;
	} else {
		targetCoords = LaunchConstants::HubPose;
	}

	if (isRedAlliance()) {
		targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
	}


	targetWhileMoving.setTargetLocation(targetCoords);
	frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassis->getEstimatedPose().Rotation());
	ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassis->getEstimatedPose().Rotation());
	frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassis->getEstimatedPose(), speed, accel);


	units::meter_t distanceToTarget = chassis->getEstimatedPose().Translation().Distance(movingGoalLocation);
 	frc::SmartDashboard::PutNumber("LaunchCommand/DistanceTarget", distanceToTarget.value());

	units::degree_t hoodAngle;
	units::turns_per_second_t shooterSpeed;

	auto launchMode = launchModeManager->getLaunchMode();
	if (launchMode == LaunchModes::Hub) {
		hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];
	} else if (launchMode == LaunchModes::Pass) {
		hoodAngle = LaunchConstants::DistanceToHoodForPass[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForPass[distanceToTarget];
	}

	if (!driver->GetHID().GetAButton()) {
		shooter->setHoodAngle(hoodAngle);
		shooter->setObjectiveVelocity(shooterSpeed * multiSupplier());
		
	} else {
		distanceToTarget = 3.1_m;
		hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];

		shooter->setHoodAngle(hoodAngle);
		shooter->setObjectiveVelocity(shooterSpeed * multiSupplier());
		
	}

	frc::SmartDashboard::PutNumber("LaunchCmd", multiSupplier());



	targetPublisher.Set(movingGoalLocation);

	


}

// Called once the command ends or is interrupted.
void LaunchCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool LaunchCommand::IsFinished() {
	return false;
}
