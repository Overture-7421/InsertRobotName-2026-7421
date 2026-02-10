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
  frc::Translation2d target = targetSupplier();
  if(isRedAlliance()){
    target = pathplanner::FlippingUtil::flipFieldPosition(target);
  }
  targetWhileMoving.setTargetLocation(target);

  

  turret->AimAtFieldPosition(chassis->getEstimatedPose(), target);
}

// Called once the command ends or is interrupted.
void LaunchCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool LaunchCommand::IsFinished() {
  return false;
}
