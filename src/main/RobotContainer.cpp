// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
	pathplanner::NamedCommands::registerCommand("LaunchCommand", std::move(LaunchCommand(&turret, &shooter, &chassis, &launchModeManager,
      [this]() -> frc::Translation2d {
        return *selectedTarget.load();
      }
    ).ToPtr()));

	pathplanner::NamedCommands::registerCommand("SwallowCommand", std::move(SwallowCommand(&intake, &processor)));
	pathplanner::NamedCommands::registerCommand("EjectCommand", std::move(processor.setProcessorCmd(ProcessorConstants::Eject)));
	pathplanner::NamedCommands::registerCommand("StopCommand", std::move(StopCommand(&intake, &processor)));



	autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);

  ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {
  
  ConfigDriverBindings();
  ConfigOperatorBindings();
}

void RobotContainer::ConfigDriverBindings() {
  chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());
	driver.Back().OnTrue(ResetHeading(&chassis));

  driver.RightTrigger().WhileTrue(LaunchCommand(&turret, &shooter, &chassis, &launchModeManager,
      [this]() -> frc::Translation2d {
        return *selectedTarget.load();
      }
    ).ToPtr()
  );

	driver.LeftTrigger().WhileTrue(SwallowCommand(&intake, &processor));
	driver.LeftTrigger().OnFalse(StopCommand(&intake, &processor));

	driver.LeftBumper().WhileTrue(processor.setProcessorCmd(ProcessorConstants::Eject));
	driver.LeftBumper().OnFalse(StopCommand(&intake, &processor));

	driver.A().WhileTrue(processor.setProcessorCmd(ProcessorConstants::ReverseProcessor));
	driver.A().OnFalse(StopCommand(&intake, &processor));

	driver.Y().WhileTrue(CloseCommand(&intake, &processor));
	driver.Y().OnFalse(StopCommand(&intake, &processor));


} 

void RobotContainer::ConfigOperatorBindings() {
  console.Button(1).OnTrue(frc2::cmd::RunOnce([this] {
    selectedTarget.store(&LaunchConstants::HubPose);
	launchModeManager.setLaunchMode(LaunchModes::Hub);
  }));

  console.Button(2).OnTrue(frc2::cmd::RunOnce([this] {
    selectedTarget.store(&LaunchConstants::LeftPass);
	launchModeManager.setLaunchMode(LaunchModes::LowPass);
  }));

  console.Button(9).OnTrue(frc2::cmd::RunOnce([this] {
    selectedTarget.store(&LaunchConstants::LeftPass);
	launchModeManager.setLaunchMode(LaunchModes::HighPass);
  }));

  console.Button(4).OnTrue(frc2::cmd::RunOnce([this] {
    selectedTarget.store(&LaunchConstants::CenterPass);
	launchModeManager.setLaunchMode(LaunchModes::HighPass);
  }));

  console.Button(5).OnTrue(frc2::cmd::RunOnce([this] {
    selectedTarget.store(&LaunchConstants::RightPass);
	launchModeManager.setLaunchMode(LaunchModes::LowPass);
  }));

  console.Button(3).OnTrue(frc2::cmd::RunOnce([this] {
    selectedTarget.store(&LaunchConstants::RightPass);
	launchModeManager.setLaunchMode(LaunchModes::HighPass);
  }));

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}

void RobotContainer::UpdateTelemetry() {
	chassis.shuffleboardPeriodic();
	turret.UpdateTelemetry();
	shooter.UpdateTelemetry();

	frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());

}

AprilTags::Config RobotContainer::railCameraRight() {
	AprilTags::Config config;
	config.cameraName = "RailRight";
	config.cameraToRobot = { 11.2_in, 3.5_in, 7.752224_in, {0_deg, -15_deg, -47.981360_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}

//Climbers no utilizaremos
AprilTags::Config RobotContainer::climberCameraLeft() {
	AprilTags::Config config;
	config.cameraName = "ClimberLeft";
	config.cameraToRobot = { -11.250259_in, 11.154470_in, 7.327807_in, {0_deg, -24_deg, -140.780604_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}

AprilTags::Config RobotContainer::climberCameraRight() {
	AprilTags::Config config;
	config.cameraName = "ClimberRight";
	config.cameraToRobot = { -9.5_in, -0.819890_in, 7.543_in, {0_deg, -15_deg, -148.525051_deg} };
	return config;
}

AprilTags::Config RobotContainer::railCameraLeft() {
	AprilTags::Config config;
	config.cameraName = "RailLeft";
	config.cameraToRobot = { 8_in, 9.2_in, 11.252224_in, {0_deg, -5_deg, -29.993788_deg} };
	return config;
}