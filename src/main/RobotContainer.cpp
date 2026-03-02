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
    )).ToPtr());

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
//   ConfigTestBindings();
}

void RobotContainer::ConfigDriverBindings() {
  chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());
	driver.Back().OnTrue(ResetHeading(&chassis));


  driver.RightTrigger().WhileTrue(LaunchCommand(&turret, &shooter, &chassis, &processor, &launchModeManager,
      [this]() -> frc::Translation2d {
        return *selectedTarget.load();
      }
    ).ToPtr());
  driver.RightBumper().OnFalse(StopCommand(&intake, &processor, &shooter));

// 	driver.LeftTrigger().WhileTrue(SwallowCommand(&intake, &processor)
// 	.AlongWith(frc2::cmd::RunOnce([this]{processor.notifyIntakeRunning(true);})
// 		).FinallyDo([this](bool interrupted) {processor.notifyIntakeRunning(false);
//       }));
// 	driver.LeftTrigger().OnFalse(StopCommand(&intake, &processor, &shooter));

// 	driver.Y().WhileTrue(CloseCommand(&intake, &processor));
// 	driver.Y().OnFalse(CloseCommand(&intake, &processor));


} 

void RobotContainer::ConfigOperatorBindings() {

// console.Button(7).WhileTrue(processor.setProcessorCmd(ProcessorConstants::ReverseProcessor));
// console.Button(7).OnFalse(StopCommand(&intake, &processor, &shooter));

// console.Button(8).WhileTrue(intake.setIntakePosition(IntakeConstants::IntakeGiver));
// console.Button(8).OnFalse(intake.setIntakePosition(IntakeConstants::IntakeSustain)); //Nose si es Sustain o Open, Luego vemos

//   console.Button(1).OnTrue(frc2::cmd::RunOnce([this] {
//     selectedTarget.store(&LaunchConstants::HubPose);
// 	launchModeManager.setLaunchMode(LaunchModes::Hub);
//   }));

//   console.Button(2).OnTrue(frc2::cmd::RunOnce([this] {
//     selectedTarget.store(&LaunchConstants::LeftPass);
// 	launchModeManager.setLaunchMode(LaunchModes::LowPass);
//   }));

//   console.Button(3).OnTrue(frc2::cmd::RunOnce([this] {
//     selectedTarget.store(&LaunchConstants::LeftPass);
// 	launchModeManager.setLaunchMode(LaunchModes::HighPass);
//   }));

//   console.Button(4).OnTrue(frc2::cmd::RunOnce([this] {
//     selectedTarget.store(&LaunchConstants::CenterPass);
// 	launchModeManager.setLaunchMode(LaunchModes::HighPass);
//   }));

//   console.Button(5).OnTrue(frc2::cmd::RunOnce([this] {
//     selectedTarget.store(&LaunchConstants::RightPass);
// 	launchModeManager.setLaunchMode(LaunchModes::LowPass);
//   }));

//   console.Button(6).OnTrue(frc2::cmd::RunOnce([this] {
//     selectedTarget.store(&LaunchConstants::RightPass);
// 	launchModeManager.setLaunchMode(LaunchModes::HighPass);
//   }));



	(isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", {0, 255, 0}).ToPtr().IgnoringDisable(true)); //Our turn
	(isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", {0, 255, 0}, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Over

	(!isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", {107, 53, 170}).ToPtr().IgnoringDisable(true)); //Inactive
	(!isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", {107, 53, 170}, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Our Turn

}

void RobotContainer::ConfigTestBindings() {
		//TEST
	test.A().WhileTrue(turret.TestCommand(180_deg));
	test.A().OnFalse(turret.TestCommand(-180_deg));

	test.RightTrigger().WhileTrue(climber.setClimberCmd(720_deg));
	test.RightTrigger().OnFalse(climber.setClimberCmd(0.0_deg));

	test.LeftTrigger().WhileTrue(climber.setClimberCmd(0_deg));
	test.LeftTrigger().OnFalse(climber.setClimberCmd(0_deg));

	// test.A().WhileTrue(frc2::cmd::Sequence(intake.setRollersVoltageCommand(6_V), frc2::cmd::Wait(0.2_s), intake.setIntakeCharacterization(-100.0_deg, 6_V)));
	// test.A().OnFalse(intake.setIntakeCharacterization(-151.0_deg, 0_V));

	// test.A().WhileTrue(climber.setClimberCmd(720_deg));
	// test.A().OnFalse(climber.setClimberCmd(300_deg));

	// test.B().WhileTrue(intake.setIntakeCharacterization(-151.0_deg, 5_V));
	// test.B().OnFalse(intake.setIntakeCharacterization(-8.0_deg, 0_V));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}

void RobotContainer::UpdateTelemetry() {
	chassis.shuffleboardPeriodic();
	turret.UpdateTelemetry();
	shooter.UpdateTelemetry();
	climber.UpdateTelemetry();

	frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());

}



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

AprilTags::Config RobotContainer::camIntakeConfig() {
	AprilTags::Config config;
	config.cameraName = "camIntake";
	config.cameraToRobot = { -3.814039_in, -6.375_in, 20.065336_in, {0_deg, -20_deg, 0.0_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}

AprilTags::Config RobotContainer::camRoboRioConfig() {
	AprilTags::Config config;
	config.cameraName = "camRoboRio";
	config.cameraToRobot = { -13.185921_in, -6.375_in, 20.065336_in, {0_deg, -160.0_deg, 0.0_deg} }; //o -160
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}