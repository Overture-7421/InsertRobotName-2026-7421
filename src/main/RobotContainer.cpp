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

static std::atomic<const frc::Translation2d*> selectedTarget{ &LaunchConstants::HubPose };

void RobotContainer::ConfigDriverBindings() {
  chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());
	driver.Back().OnTrue(ResetHeading(&chassis));

  // driver.RightBumper().WhileTrue(LaunchCommand(&turret, &shooter, &chassis,
  //     []() -> frc::Translation2d {
  //       return *selectedTarget.load();
  //     }
  //   ).ToPtr()
  // );

  driver.A().WhileTrue(turret.TestCommand(90_deg));
  driver.A().OnFalse(turret.TestCommand(0_deg));
} 

void RobotContainer::ConfigOperatorBindings() {
  // console.Button(1).OnTrue(frc2::cmd::RunOnce([&] {
  //   selectedTarget.store(&LaunchConstants::HubPose);

  // }));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

	return autoChooser.GetSelected();
}