// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveCommand.h"
#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>
#include <cmath>
#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>
#include "Commands/LaunchCommand/LaunchConstants.h"

DriveCommand::DriveCommand(Chassis* chassis, OverXboxController* gamepad, Processor* processor) : headingSpeedsHelper{ headingController,
		chassis } {
	this->chassis = chassis;
	this->gamepad = gamepad;
	this->processor = processor;
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ chassis });
}

// Called when the command is initially scheduled.
void DriveCommand::Initialize() {
	if (isRedAlliance()) {
		allianceMulti = -1;
	} else {
		allianceMulti = 1;
	}
}

// Called repeatedly when this Command is scheduled to run
void DriveCommand::Execute() {
	frc::Rotation2d targetAngle {gamepad->getRightStickDirection()};

	if (allianceMulti == -1) {
		targetAngle = targetAngle.RotateBy( {180_deg});
	}

	double squares = sqrt(gamepad->GetRightY() * gamepad->GetRightY() + gamepad->GetRightX() * gamepad->GetRightX());

	if (squares > 0.71) {
		if (speedHelperMoved == false) {
			speedHelperMoved = true;
			chassis->enableSpeedHelper(&headingSpeedsHelper);
		}
	} else if (speedHelperMoved == true) {
		speedHelperMoved = false;
		chassis->disableSpeedHelper();
	}
	 
	headingSpeedsHelper.setTargetAngle(targetAngle);

	auto xSpeed = Utils::ApplyAxisFilter(allianceMulti * -gamepad->GetHID().GetRawAxis(1), 0.06, 0.5)
		* chassis->getMaxModuleSpeed() * slowMulti;
	auto ySpeed = Utils::ApplyAxisFilter(allianceMulti * -gamepad->GetHID().GetRawAxis(0), 0.06, 0.5)
		* chassis->getMaxModuleSpeed() * slowMulti;

	if (gamepad->GetHID().GetRightBumperButton()) {
		xSpeed = units::meters_per_second_t(std::clamp(xSpeed.value(), -shootWhileMoveMaxSpeed.value(), shootWhileMoveMaxSpeed.value()));
		ySpeed = units::meters_per_second_t(std::clamp(ySpeed.value(), -shootWhileMoveMaxSpeed.value(), shootWhileMoveMaxSpeed.value()));
	}

	auto rotationSpeed = (gamepad->getTwist() * 1_tps);
	// auto rotationSpeed = (Utils::ApplyAxisFilter(gamepad->GetRightX(), 0.06, 0.75) * -1_tps); //-0.7
	// frc::SmartDashboard::PutNumber("DriveCommand/RotationSpeed", rotationSpeed.value());

	frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xInput.Calculate(xSpeed), yInput.Calculate(ySpeed), rotationSpeed,
		chassis->getEstimatedPose().Rotation());

	chassis->setTargetSpeeds(speeds);
}

// Called once the command ends or is interrupted.
void DriveCommand::End(bool interrupted) {
	chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool DriveCommand::IsFinished() {
	return false;
}