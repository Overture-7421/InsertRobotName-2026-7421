// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"

LaunchCommand::LaunchCommand(Shooter* shooter,Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver) : multiSupplier(std::move(multiSupplier)), headingSpeedsHelper({7, 0, 0.5,{1200_deg_per_s, 1200_deg_per_s_sq}}, chassis) {
	this->shooter = shooter;
	this->hood = hood;
	this->chassis = chassis;
	this->launchModeManager = launchModeManager;
	this->driver = driver;

	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ shooter, hood});
}

// Called when the command is initially scheduled.
void LaunchCommand::Initialize() { }

// Called repeatedly when this Command is scheduled to run
void LaunchCommand::Execute() {
	auto launchMode = launchModeManager->getLaunchMode();
	const frc::Pose2d& chassisPose = chassis->getEstimatedPose();

	frc::Translation2d targetCoords;
	if (launchMode == LaunchModes::Pass) {
		targetCoords = passTargetSwitcher.GetPassTarget(chassisPose);
	} else {
		targetCoords = LaunchConstants::HubPose;
	}

	if (isRedAlliance()) {
		targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
	}


	targetWhileMoving.setTargetLocation(targetCoords);
	frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassisPose.Rotation());
	ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassisPose.Rotation());
	frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassisPose, speed, accel);

	if(driver->GetHID().GetRightBumperButton()){
		if (speedHelperMoved == false) {
			speedHelperMoved = true;
			chassis->enableSpeedHelper(&headingSpeedsHelper);
		}
	} else if (speedHelperMoved == true) {
		speedHelperMoved = false;
		chassis->disableSpeedHelper();
	}
	frc::Rotation2d targetAngle((chassisPose.X() - movingGoalLocation.X()).value(), (chassisPose.Y() - movingGoalLocation.Y()).value());
  	headingSpeedsHelper.setTargetAngle(targetAngle);


	units::meter_t distanceToTarget = chassisPose.Translation().Distance(movingGoalLocation);
 	frc::SmartDashboard::PutNumber("LaunchCommand/DistanceTarget", distanceToTarget.value());

	units::degree_t hoodAngle;
	units::turns_per_second_t shooterSpeed;

	if (launchMode == LaunchModes::Hub) {
		hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];
	} else if (launchMode == LaunchModes::Pass) {
		hoodAngle = LaunchConstants::DistanceToHoodForPass[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForPass[distanceToTarget];
	}
	
	if (!driver->GetHID().GetAButton()) {
		hood->setHoodAngle(hoodAngle);
		shooter->setObjectiveVelocity(shooterSpeed * multiSupplier());
		
	} else {
		distanceToTarget = 3.1_m;
		hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];

		hood->setHoodAngle(hoodAngle);
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
