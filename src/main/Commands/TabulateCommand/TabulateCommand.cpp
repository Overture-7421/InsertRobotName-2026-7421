// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TabulateCommand.h"

TabulateCommand::TabulateCommand(Shooter* shooter, Chassis* chassis, std::function<frc::Translation2d()> targetSupplier) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->shooter = shooter;
  this->chassis = chassis;
  this->targetSupplier = std::move(targetSupplier);

  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void TabulateCommand::Initialize() {

  frc::SmartDashboard::PutNumber("Tabulate/HoodAngle", shooter->getHoodAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVel", shooter->getShooterVelocity().value());


}

// Called repeatedly when this Command is scheduled to run
void TabulateCommand::Execute() {
    frc::Translation2d targetCoords = targetSupplier();

  if(isRedAlliance()){
    targetCoords = pathplanner::FlippingUtil::flipFieldPosition(targetCoords);
  }

  units::meter_t distanceToTarget = chassis->getEstimatedPose().Translation().Distance(targetCoords);

  frc::SmartDashboard::PutNumber("Tabulate/Distance", distanceToTarget.value());
	frc::SmartDashboard::PutNumber("Tabulate/HoodAngleCurrent", shooter->getHoodAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVelCurrent", shooter->getShooterVelocity().value());

  units::degree_t hoodAngle{ frc::SmartDashboard::GetNumber("Tabulate/HoodAngle", shooter->getHoodAngle().value()) };
	double targetVel = frc::SmartDashboard::GetNumber("Tabulate/ShooterVel", 0.0);

  shooter->setHoodAngle(hoodAngle);
  shooter->setObjectiveVelocity(targetVel * 1_tps);
}

// Called once the command ends or is interrupted.
void TabulateCommand::End(bool interrupted) {
  shooter->setShooterVelocityCommand(0.0_tps);
}

// Returns true when the command should end.
bool TabulateCommand::IsFinished() {
  return false;
}
