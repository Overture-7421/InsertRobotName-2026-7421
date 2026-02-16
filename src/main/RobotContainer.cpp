// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here


  // Configure the button bindings
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);

  ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {
  ConfigDriverBindings();
}

void RobotContainer::ConfigDriverBindings() {
  driver.A().WhileTrue(intake.setIntakePosition(intakeConstants::IntakeOpen));
  driver.A().OnFalse(intake.setIntakePosition(intakeConstants::IntakeStow));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}