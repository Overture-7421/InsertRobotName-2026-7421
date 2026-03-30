// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"

LaunchCommand::LaunchCommand(Shooter* shooter,Hood* hood, Chassis* chassis, Intake* intake, Processor* processor, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver) : multiSupplier(std::move(multiSupplier)), headingSpeedsHelper({7, 0, 0.5,{1200_deg_per_s, 1200_deg_per_s_sq}}, chassis) {
	this->shooter = shooter;
	this->hood = hood;
	this->chassis = chassis;
	this->intake = intake;
	this->processor = processor;
	this->launchModeManager = launchModeManager;
	this->driver = driver;

	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ shooter, hood, intake, processor});
}

// Called when the command is initially scheduled.
void LaunchCommand::Initialize() { 
	chassis->enableSpeedHelper(&headingSpeedsHelper);

	intake->intakeSlowModeFilter.Reset(intake->getIntakePosition());
}

// Called repeatedly when this Command is scheduled to run
void LaunchCommand::Execute() {
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


	targetWhileMoving.setTargetLocation(targetCoords);
	frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassis->getEstimatedPose().Rotation());
	ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassis->getEstimatedPose().Rotation());
	frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassis->getEstimatedPose(), speed, accel);

	frc::Rotation2d targetAngle((chassis->getEstimatedPose().X() - movingGoalLocation.X()).value(), (chassis->getEstimatedPose().Y() - movingGoalLocation.Y()).value());
  	headingSpeedsHelper.setTargetAngle(targetAngle);


	units::meter_t distanceToTarget = chassis->getEstimatedPose().Translation().Distance(movingGoalLocation);
 	frc::SmartDashboard::PutNumber("LaunchCommand/DistanceTarget", distanceToTarget.value());

	units::degree_t hoodAngle;
	units::turns_per_second_t shooterSpeed;

	if (chassis->getEstimatedPose().X() > 4.129_m) {
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
	units::degree_t chassisError = units::math::abs(targetAngle.Degrees() - chassis->getEstimatedPose().Rotation().Degrees());
	if(shooter->isShooterAtVelocity(shooterSpeed * multiSupplier()) && hood->isHoodAtAngle(hoodAngle) && chassisError < 2_deg){
		intake->setIntakeDistance(intake->intakeSlowModeFilter.Calculate(IntakeConstants::IntakeClose.intake));
		processor->setProcessorVoltages(ProcessorConstants::Eject);
	} else {
		processor->setProcessorVoltages(ProcessorConstants::StopProcessor);
	}

	frc::SmartDashboard::PutNumber("LaunchCmd", multiSupplier());
	targetPublisher.Set(movingGoalLocation);
}

// Called once the command ends or is interrupted.
void LaunchCommand::End(bool interrupted) {
	chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool LaunchCommand::IsFinished() {
	return false;
}
