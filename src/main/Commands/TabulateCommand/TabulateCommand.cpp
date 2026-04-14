// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TabulateCommand.h"

TabulateCommand::TabulateCommand(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager) : headingSpeedsHelper(headingController, chassis) {
	this->shooter = shooter;
	this->hood = hood;
	this->chassis = chassis;
	this->launchModeManager = launchModeManager;

	AddRequirements({ shooter, hood});
}

// Called when the command is initially scheduled.
void TabulateCommand::Initialize() {

	frc::SmartDashboard::PutNumber("Tabulate/HoodAngle", hood->getHoodAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVel", shooter->getShooterVelocity().value());
	chassis->enableSpeedHelper(&headingSpeedsHelper);


}

// Called repeatedly when this Command is scheduled to run
void TabulateCommand::Execute() {
	auto launchMode = launchModeManager->getLaunchMode();
	const frc::Pose2d& chassisPose = chassis->getEstimatedPose();\
	bool redAlliance = isRedAlliance();
	frc::Translation2d targetCoords;
	
	if(launchMode == LaunchModes::Pass) {
		targetCoords = passTargetSwitcher.GetPassTarget(chassisPose, redAlliance);
	} else {
		targetCoords = LaunchConstants::HubPose;
	}
	if (redAlliance) {
		targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
	}

	frc::Rotation2d targetAngle((chassisPose.X() - targetCoords.X()).value(), (chassisPose.Y() - targetCoords.Y()).value());
  	headingSpeedsHelper.setTargetAngle(targetAngle);

	units::meter_t distanceToTarget = (chassis->getEstimatedPose()).Translation().Distance(targetCoords);

	frc::SmartDashboard::PutNumber("Tabulate/Distance", distanceToTarget.value());
	frc::SmartDashboard::PutNumber("Tabulate/HoodAngleCurrent", hood->getHoodAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVelCurrent", shooter->getShooterVelocity().value());

	units::degree_t hoodAngle{ frc::SmartDashboard::GetNumber("Tabulate/HoodAngle", hood->getHoodAngle().value()) };
	double targetVel = frc::SmartDashboard::GetNumber("Tabulate/ShooterVel", 0.0);

	hood->setHoodAngle(hoodAngle);
	shooter->setObjectiveVelocity(targetVel * 1_tps);

	frc::SmartDashboard::PutBoolean("Tabulate/AtPosition/ShooterIsAtVelocity", shooter->isShooterAtVelocity());
	frc::SmartDashboard::PutBoolean("Tabulate/AtPosition/HoodIsHoodAngle", hood->isHoodAtAngle());


}

// Called once the command ends or is interrupted.
void TabulateCommand::End(bool interrupted) {
	shooter->setObjectiveVelocity(0.0_tps);
	chassis->disableSpeedHelper();

}

// Returns true when the command should end.
bool TabulateCommand::IsFinished() {
	return false;
}
