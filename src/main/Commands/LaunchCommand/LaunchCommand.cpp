// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"

LaunchCommand::LaunchCommand(Turret* turret, Shooter* shooter, Chassis* chassis, Processor* processor,  LaunchModeManager* launchModeManager, std::function<frc::Translation2d()> targetSupplier) {
  this->turret = turret;
  this->shooter = shooter;
  this->chassis = chassis;
  this->processor = processor;
  this->launchModeManager = launchModeManager;
  this->targetSupplier = std::move(targetSupplier);

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({turret, shooter, processor});
}

// Called when the command is initially scheduled.
void LaunchCommand::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void LaunchCommand::Execute() {
  frc::Translation2d targetCoords = targetSupplier();

  frc::SmartDashboard::PutNumber("LaunchTarget_X", targetCoords.X().value());
  frc::SmartDashboard::PutNumber("LaunchTarget_Y", targetCoords.Y().value());

  if(isRedAlliance()){
    targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
  }

  targetWhileMoving.setTargetLocation(targetCoords);
  frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassis->getEstimatedPose().Rotation());
  ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassis->getEstimatedPose().Rotation());
  frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassis->getEstimatedPose(), speed, accel);

  turret->AimAtFieldPosition(chassis->getEstimatedPose(), movingGoalLocation);

  units::meter_t distanceToTarget = chassis->getEstimatedPose().Translation().Distance(movingGoalLocation);
  frc::SmartDashboard::PutNumber("Launch_Distance_m", distanceToTarget.value());

  units::degree_t hoodAngle;
  units::turns_per_second_t shooterSpeed;

  auto launchMode = launchModeManager->getLaunchMode();
  if(launchMode == LaunchModes::Hub){
    hoodAngle = LaunchConstants::DistanceToHoodForHub[distanceToTarget];
    shooterSpeed = LaunchConstants::DistanceToShooterForHub[distanceToTarget];
  } else if (launchMode == LaunchModes::HighPass){
    hoodAngle = LaunchConstants::DistanceToHoodForHighPass[distanceToTarget];
    shooterSpeed = LaunchConstants::DistanceToShooterForHighPass[distanceToTarget];
  } else {
    hoodAngle = LaunchConstants::DistanceToHoodForLowPass[distanceToTarget];
    shooterSpeed = LaunchConstants::DistanceToShooterForLowPass[distanceToTarget];
  }

  shooter->setHoodAngle(hoodAngle);
  shooter->setObjectiveVelocity(shooterSpeed);

  if(turret->isAimAtFieldPosition(chassis->getEstimatedPose(), movingGoalLocation) && shooter->isShooterAtVelocity(shooterSpeed) && shooter->isHoodAtAngle(hoodAngle) && !frc::DriverStation::IsAutonomous()){
    processor->setSpindexerPasserVoltage(ProcessorConstants::Eject);
  }

  frc::SmartDashboard::PutBoolean("AtPosition/ShooterIsAtVelocity", shooter->isShooterAtVelocity(shooterSpeed));
  frc::SmartDashboard::PutBoolean("AtPosition/HoodIsHoodAngle", shooter->isHoodAtAngle(hoodAngle));
  frc::SmartDashboard::PutBoolean("AtPosition/TurretIsAtFieldPos", turret->isAimAtFieldPosition(chassis->getEstimatedPose(), movingGoalLocation));

  
}
  
// Called once the command ends or is interrupted.
void LaunchCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool LaunchCommand::IsFinished() {
  return false;
}
