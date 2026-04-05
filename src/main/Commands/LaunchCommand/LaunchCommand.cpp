// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"
#include <frc/Timer.h>

LaunchCommand::LaunchCommand(Shooter* shooter,Hood* hood, Chassis* chassis, Intake* intake, Processor* processor, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver) : headingSpeedsHelper(headingController, chassis), multiSupplier(std::move(multiSupplier)) {
	this->shooter = shooter;
	this->hood = hood;
	this->chassis = chassis;
	this->intake = intake;
	this->processor = processor;
	this->launchModeManager = launchModeManager;
	this->driver = driver;

	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ shooter, hood, processor}); //Intake is crashing
}

// Called when the command is initially scheduled.
void LaunchCommand::Initialize() { 
	chassis->enableSpeedHelper(&headingSpeedsHelper);
	intake->intakeSlowModeFilter.Reset(intake->getIntakePosition());
}

// Called repeatedly when this Command is scheduled to run
void LaunchCommand::Execute() {
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
		distanceToTarget = 3.1_m;
		hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
		shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];

		hood->setHoodAngle(hoodAngle);
		shooter->setObjectiveVelocity(shooterSpeed * multiSupplier());
		
	}

	//Eject when at position
	const units::time::second_t now = frc::Timer::GetFPGATimestamp();
	units::degree_t chassisError = units::math::abs(targetAngle.Degrees() - chassisPose.Rotation().Degrees());
	frc::SmartDashboard::PutNumber("LaunchCommand/ChassisError", chassisError.value());
	if((shooter->isShooterAtVelocity() || shooter->getState() == ShooterState::Holding) && hood->isHoodAtAngle() && chassisError < 3.25_deg){

		if (!inTargetState) {
			inTargetState = true;
			startedClosing = false;
			enterTimestamp = now;
			shooter->Hold();
		}

		intake->setRollersVoltage(IntakeConstants::IntakeOpen.rollers);
		processor->setProcessorVoltages(ProcessorConstants::Eject);

		// after 0.6s start to close
		if (!startedClosing && (now - enterTimestamp) > 0.6_s) {
			intake->intakeSlowModeFilter.Reset(intake->getIntakePosition());
			startedClosing = true;
		}

		if (startedClosing) {
			intake->setIntakeDistance(intake->intakeSlowModeFilter.Calculate(IntakeConstants::IntakeClose.intake));
		}

	} else {
		inTargetState = false;
		startedClosing = false;
		shooter->Release();
		processor->setProcessorVoltages(ProcessorConstants::Stop);
	}


	if(intake->getIntakePosition() < IntakeConstants::RollersShouldNotBeMoving ){
		intake->setRollersVoltage(IntakeConstants::IntakeClose.rollers);
	}

	frc::SmartDashboard::PutNumber("LaunchCmd", multiSupplier());
	targetPublisher.Set(movingGoalLocation);
}

// Called once the command ends or is interrupted.
void LaunchCommand::End(bool interrupted) {
	chassis->disableSpeedHelper();
	shooter->Release();
}

// Returns true when the command should end.
bool LaunchCommand::IsFinished() {
	return false;
}