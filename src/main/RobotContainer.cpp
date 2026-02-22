// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
	pathplanner::NamedCommands::registerCommand("LaunchCommand", std::move(LaunchCommand(&turret, &shooter, &chassis, &processor, &launchModeManager,
      [this]() -> frc::Translation2d {
        return *selectedTarget.load();
      }
    ).BeforeStarting(frc2::cmd::RunOnce([this] {
          // desactivar auto-preload justo antes de ejecutar LaunchCommand
          processor.setAutoPreloadEnabled(false);
      }))
      .FinallyDo([this](bool interrupted) {
          // se ejecuta cuando LaunchCommand termine o sea interrumpido
          processor.setAutoPreloadEnabled(true);
      })));

	pathplanner::NamedCommands::registerCommand("SwallowCommand", std::move(SwallowCommand(&intake, &processor)
	.AlongWith(frc2::cmd::RunOnce([this]{processor.notifyIntakeRunning(true);})
		).FinallyDo([this](bool interrupted) {processor.notifyIntakeRunning(false);
      })));
	pathplanner::NamedCommands::registerCommand("EjectCommand", std::move(processor.setProcessorCmd(ProcessorConstants::Eject)));
	pathplanner::NamedCommands::registerCommand("StopCommand", std::move(StopCommand(&intake, &processor, &shooter)));



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

  driver.RightTrigger().WhileTrue(LaunchCommand(&turret, &shooter, &chassis, &processor, &launchModeManager,
      [this]() -> frc::Translation2d {
        return *selectedTarget.load();
      }
    ).BeforeStarting(frc2::cmd::RunOnce([this] {
          // desactivar auto-preload justo antes de ejecutar LaunchCommand
          processor.setAutoPreloadEnabled(false);
      }))
      .FinallyDo([this](bool interrupted) {
          // se ejecuta cuando LaunchCommand termine o sea interrumpido
          processor.setAutoPreloadEnabled(true);
      })
    );
  driver.RightBumper().OnFalse(StopCommand(&intake, &processor, &shooter));

	driver.LeftTrigger().WhileTrue(SwallowCommand(&intake, &processor)
	.AlongWith(frc2::cmd::RunOnce([this]{processor.notifyIntakeRunning(true);})
		).FinallyDo([this](bool interrupted) {processor.notifyIntakeRunning(false);
      }));
	driver.LeftTrigger().OnFalse(StopCommand(&intake, &processor, &shooter));

	driver.A().WhileTrue(processor.setProcessorCmd(ProcessorConstants::ReverseProcessor));
	driver.A().OnFalse(StopCommand(&intake, &processor, &shooter));

	driver.Y().WhileTrue(CloseCommand(&intake, &processor));
	driver.Y().OnFalse(CloseCommand(&intake, &processor));



} 

void RobotContainer::ConfigOperatorBindings() {

	autoWin.OnTrue(LedsWinAuto(&leds));
	autoLose.OnTrue(LedsLoseAuto(&leds));


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

// AprilTags::Config RobotContainer::camIntakeConfig() {  	Not Defined Yet
// 	AprilTags::Config config;
// 	config.cameraName = "camIntake";
// 	config.cameraToRobot = { 11.2_in, 3.5_in, 7.752224_in, {0_deg, -15_deg, -47.981360_deg} };
// 	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
// 	return config;
// }

AprilTags::Config RobotContainer::camStorageConfig() {
	AprilTags::Config config;
	config.cameraName = "camStorage";
	config.cameraToRobot = { 4.433894_in, 13.068914_in, 10.670724_in, {0_deg, -10.127476_deg, 0.0_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}

AprilTags::Config RobotContainer::camRadioConfig() {
	AprilTags::Config config;
	config.cameraName = "camRadio";
	config.cameraToRobot = { -5.05684_in, -13.144803_in, 7.411252_in, {0_deg, -15.000170_deg, 0.0_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}

AprilTags::Config RobotContainer::camRoboRioConfig() {
	AprilTags::Config config;
	config.cameraName = "camRoboRio";
	config.cameraToRobot = { -13.531365_in, -4.438194_in, 6.637019_in, {0_deg, -20.125_deg, 0.0_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}