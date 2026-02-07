// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"

LaunchCommand::LaunchCommand(Turret* turret, Shooter* shooter, Chassis* chassis, frc::Translation2d targetObjective) {
  this->turret = turret;
  this->shooter = shooter;
  this->chassis = chassis;
  this->targetObjective = targetObjective;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({turret, shooter});
}

// Called when the command is initially scheduled.
void LaunchCommand::Initialize() {

  if(isRedAlliance()){
    targetObjective = pathplanner::FlippingUtil::flipFieldPosition(targetObjective);
  } else {
    targetObjective = targetObjective;
  }

}

// Called repeatedly when this Command is scheduled to run
void LaunchCommand::Execute() {
  turret->AimAtFieldPosition(chassis->getEstimatedPose(), targetObjective);
}

// Called once the command ends or is interrupted.
void LaunchCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool LaunchCommand::IsFinished() {
  return false;
}
