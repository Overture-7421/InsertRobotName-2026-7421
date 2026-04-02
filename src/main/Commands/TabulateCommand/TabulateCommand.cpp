// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TabulateCommand.h"

TabulateCommand::TabulateCommand(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager) {
	// Use addRequirements() here to declare subsystem dependencies.
	this->shooter = shooter;
	this->hood = hood;
	this->chassis = chassis;
	// this->targetSupplier = std::move(targetSupplier);
	this->launchModeManager = launchModeManager;

	AddRequirements({ shooter, hood});
}

// Called when the command is initially scheduled.
void TabulateCommand::Initialize() {

	frc::SmartDashboard::PutNumber("Tabulate/HoodAngle", hood->getHoodAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVel", shooter->getShooterVelocity().value());


}

// Called repeatedly when this Command is scheduled to run
void TabulateCommand::Execute() {
	// auto launchMode = launchModeManager->getLaunchMode();
	
	frc::Translation2d targetCoords;
	if(chassis->getEstimatedPose().X() > 4.129_m){
		if(chassis->getEstimatedPose().Y() > 4.2_m){
			targetCoords = LaunchConstants::LeftPass;
		} else if(chassis->getEstimatedPose().Y() < 3.8_m){
			targetCoords = LaunchConstants::RightPass;
		}
	} else {
		targetCoords = LaunchConstants::HubPose;
	}

	if (isRedAlliance()) {
		targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
	}


	units::meter_t distanceToTarget = (chassis->getEstimatedPose()).Translation().Distance(targetCoords);

	frc::SmartDashboard::PutNumber("Tabulate/Distance", distanceToTarget.value());
	frc::SmartDashboard::PutNumber("Tabulate/HoodAngleCurrent", hood->getHoodAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVelCurrent", shooter->getShooterVelocity().value());

	units::degree_t hoodAngle{ frc::SmartDashboard::GetNumber("Tabulate/HoodAngle", hood->getHoodAngle().value()) };
	double targetVel = frc::SmartDashboard::GetNumber("Tabulate/ShooterVel", 0.0);

	//turret->AimAtFieldPosition(chassis->getEstimatedPose(), targetCoords);

	hood->setHoodAngle(hoodAngle);
	shooter->setObjectiveVelocity(targetVel * 1_tps);

	frc::SmartDashboard::PutBoolean("Tabulate/AtPosition/ShooterIsAtVelocity", shooter->isShooterAtVelocity());
	frc::SmartDashboard::PutBoolean("Tabulate/AtPosition/HoodIsHoodAngle", hood->isHoodAtAngle());
	//frc::SmartDashboard::PutBoolean("Tabulate/AtPosition/TurretIsAtFieldPos", turret->isAimAtFieldPosition(chassis->getEstimatedPose(), targetCoords));
}

// Called once the command ends or is interrupted.
void TabulateCommand::End(bool interrupted) {
	shooter->setObjectiveVelocity(0.0_tps);
}

// Returns true when the command should end.
bool TabulateCommand::IsFinished() {
	return false;
}
