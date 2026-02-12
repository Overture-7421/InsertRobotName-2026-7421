// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"

LaunchCommand::LaunchCommand(Turret* turret, Shooter* shooter, Chassis* chassis, std::function<frc::Translation2d()> targetSupplier) {
  this->turret = turret;
  this->shooter = shooter;
  this->chassis = chassis;
  this->targetSupplier = std::move(targetSupplier);
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({turret, shooter});
}

// Called when the command is initially scheduled.
void LaunchCommand::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void LaunchCommand::Execute() {
  frc::Translation2d targetCoords = targetSupplier();
  if(isRedAlliance()){
    targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
  }

  targetWhileMoving.setTargetLocation(targetCoords);
  frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassis->getEstimatedPose().Rotation());
  ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassis->getEstimatedPose().Rotation());
  frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassis->getEstimatedPose(), speed, accel);

  turret->AimAtFieldPosition(chassis->getEstimatedPose(), movingGoalLocation);

  units::meter_t distanceToTarget = chassis->getEstimatedPose().Translation().Distance(movingGoalLocation);
  units::degree_t hoodAngle = LaunchConstants::DistanceToHood[distanceToTarget];
  units::turns_per_second_t shooterSpeed = LaunchConstants::DistanceToShooter[distanceToTarget];

  shooter->setHoodAngle(hoodAngle);
  shooter->setObjectiveVelocity(shooterSpeed);
}

// Called once the command ends or is interrupted.
void LaunchCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool LaunchCommand::IsFinished() {
  return false;
}
