// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer(){

	//Sujto a cambio
	//pathplanner::NamedCommands::registerCommand("SwallowCommand", std::move(SwallowCommand(&intake)));
	pathplanner::NamedCommands::registerCommand("IntakeSustain", std::move(intake.setIntakeCmd(IntakeConstants::IntakeSustainWithoutRollers)));
	pathplanner::NamedCommands::registerCommand("EjectCommand", std::move(processor.setProcessorCmd(ProcessorConstants::Eject)));
	pathplanner::NamedCommands::registerCommand("StopIndexer", std::move(frc2::cmd::Parallel(processor.setProcessorCmd(ProcessorConstants::StopProcessor), intake.setRollersCmd(IntakeConstants::RollersStop))));
	pathplanner::NamedCommands::registerCommand("IntakeGiver", std::move(intake.setIntakeCmd(IntakeConstants::IntakeGiver).WithTimeout(0.5_s)));
	pathplanner::NamedCommands::registerCommand("ShooterStop", std::move(shooter.setShooterVelocityCommand(0_tps)));


	autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);

	ConfigureBindings();

	launchCommand = std::make_unique<LaunchCommand>(&shooter, &chassis, &launchModeManager, [this] {return launchShooterMulti;}, &driver);

}

void RobotContainer::ConfigureBindings() {
	ConfigDriverBindings();
	ConfigOperatorBindings();
	ConfigConsoleBindings();
	ConfigTestBindings();
}

void RobotContainer::ConfigDriverBindings() {
	chassis.SetDefaultCommand(DriveCommand(&chassis, &driver, &processor).ToPtr());
	driver.Back().OnTrue(ResetHeading(&chassis));

	//driver.LeftTrigger().WhileTrue(SwallowCommand(&intake));
	driver.LeftTrigger().OnFalse(intake.setIntakeCmd(IntakeConstants::IntakeSustainWithoutRollers));


	driver.RightTrigger().WhileTrue(processor.setProcessorCmd(ProcessorConstants::Eject));
	driver.RightTrigger().OnFalse(processor.setProcessorCmd(ProcessorConstants::StopProcessor));

	// driver.A().WhileTrue(frc2::cmd::Parallel(turret.setTargetAngle(0_deg))) 3.1distance

	// driver.RightTrigger().WhileTrue(EjectCommand(&processor, &turret, &intake).ToPtr());
	// driver.RightTrigger().OnFalse(frc2::cmd::Parallel(processor.setProcessorCmd(ProcessorConstants::StopProcessor), intake.setRollersCmd(IntakeConstants::RollersStop)));

	// 	driver.Y().WhileTrue(CloseCommand(&intake, &processor));
	// 	driver.Y().OnFalse(CloseCommand(&intake, &processor));


	// driver.LeftBumper().ToggleOnTrue(TabulateCommand(&shooter, &chassis, &turret, &launchModeManager).ToPtr());



	(isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", { 0, 255, 0 }).ToPtr().IgnoringDisable(true)); //Our turn
	(isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", { 0, 255, 0 }, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Over

	(!isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", { 107, 53, 170 }).ToPtr().IgnoringDisable(true)); //Inactive
	(!isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", { 107, 53, 170 }, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Our Turn

}

void RobotContainer::ConfigOperatorBindings() {

	oprtr.LeftBumper().WhileTrue(intake.setIntakeCmd(IntakeConstants::IntakeGiver));
	oprtr.LeftBumper().OnFalse(intake.setIntakeCmd(IntakeConstants::IntakeSustain));

	oprtr.RightBumper().WhileTrue(processor.setProcessorCmd(ProcessorConstants::ReverseProcessor));
	oprtr.RightBumper().OnFalse(processor.setProcessorCmd(ProcessorConstants::StopProcessor));

	oprtr.A().OnTrue(frc2::cmd::RunOnce([this] {
		// selectedTarget.store(&LaunchConstants::HubPose);
		launchModeManager.setSideMode(SideMode::Hub);
		launchModeManager.setLaunchMode(LaunchModes::Hub);
	}));

	oprtr.X().OnTrue(frc2::cmd::RunOnce([this] {
		// selectedTarget.store(&LaunchConstants::LeftPass);
		launchModeManager.setSideMode(SideMode::Left);
		launchModeManager.setLaunchMode(LaunchModes::Pass);
	}));

	oprtr.B().OnTrue(frc2::cmd::RunOnce([this] {
		// selectedTarget.store(&LaunchConstants::RightPass);
		launchModeManager.setSideMode(SideMode::Right);
		launchModeManager.setLaunchMode(LaunchModes::Pass);
	}));

}

void RobotContainer::ConfigConsoleBindings() {
	console.Button(3).WhileTrue(processor.setProcessorCmd(ProcessorConstants::ReverseProcessor));
	console.Button(3).OnFalse(processor.setProcessorCmd(ProcessorConstants::StopProcessor));

	console.Button(6).OnTrue(intake.setIntakeCmd(IntakeConstants::IntakeGiver).WithTimeout(0.1_s).Unless([this] {return driver.LeftTrigger().Get();}));
	console.Button(6).OnFalse(intake.setIntakeCmd(IntakeConstants::SustainAfterGiver).WithTimeout(0.1_s).Unless([this] {return driver.LeftTrigger().Get();}));

	console.Button(2).OnTrue(frc2::cmd::RunOnce([this] {
		// selectedTarget.store(&LaunchConstants::HubPose);
		launchModeManager.setLaunchMode(LaunchModes::Hub);
		launchModeManager.setSideMode(SideMode::Hub);
	}));

	console.Button(1).OnTrue(frc2::cmd::RunOnce([this] {
		// selectedTarget.store(&LaunchConstants::LeftPass);
		launchModeManager.setLaunchMode(LaunchModes::Pass);
		launchModeManager.setSideMode(SideMode::Left);
	}));

	console.Button(12).OnTrue(frc2::cmd::RunOnce([this] {
		// selectedTarget.store(&LaunchConstants::RightPass);
		launchModeManager.setLaunchMode(LaunchModes::Pass);
		launchModeManager.setSideMode(SideMode::Right);
	}));

	console.Button(5).OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti += 0.03;
	}));

	console.Button(11).OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti -= 0.03;
	}));
}

void RobotContainer::ConfigTestBindings() {
	//TEST
// test.A().WhileTrue(turret.TestCommand(180_deg));
// test.A().OnFalse(turret.TestCommand(-180_deg));

// test.RightTrigger().WhileTrue(climber.setClimberCmd(720_deg));
// test.RightTrigger().OnFalse(climber.setClimberCmd(0.0_deg));

// test.LeftTrigger().WhileTrue(climber.setClimberCmd(0_deg));
// test.LeftTrigger().OnFalse(climber.setClimberCmd(0_deg));

	// test.A().WhileTrue(shooter.setShooterVelocityCommand(36_tps));
	// test.A().OnFalse(shooter.setShooterVelocityCommand(27_tps));

	// test.A().WhileTrue(frc2::cmd::Sequence(intake.setRollersVoltageCommand(6_V), frc2::cmd::Wait(0.2_s), intake.setIntakeCharacterization(-100.0_deg, 6_V)));
	// test.A().OnFalse(intake.setIntakeCharacterization(-151.0_deg, 0_V));

	// test.B().WhileTrue(intake.setIntakeCharacterization(120.0_deg, 0_V));
	// test.B().OnFalse(intake.setIntakeCharacterization(30.0_deg, 0_V));

	// test.A().WhileTrue(shooter.setHoodAngleCommand(32.0_deg));
	// test.A().OnFalse(shooter.setHoodAngleCommand(3.0_deg));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}

void RobotContainer::UpdateTelemetry() {
	chassis.shuffleboardPeriodic();
	//turret.UpdateTelemetry();
	shooter.UpdateTelemetry();
	intake.UpdateTelemetry();
	// climber.UpdateTelemetry();

	frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());

	Logging::WriteDouble("Shoot Multiplier", launchShooterMulti);

}



AprilTags::Config RobotContainer::camStorageConfig() {
	AprilTags::Config config;
	config.cameraName = "camStorage";
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -8.5_in, -6.970297_in, 18.080053_in, {0_deg, -27_deg, 90.0_deg} };};
	return config;
}

AprilTags::Config RobotContainer::camRadioConfig() {
	AprilTags::Config config;
	config.cameraName = "camRadio";
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -8.5_in , -10.279703_in, 18.080053_in, {0_deg, -27_deg, -90.0_deg} };};
	return config;
}

AprilTags::Config RobotContainer::camIntakeConfig() {
	AprilTags::Config config;
	config.cameraName = "camIntake";
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -3.814079_in, -6.375_in, 18.065336_in, {0_deg, -20_deg, 0.0_deg} };};
	return config;
}

AprilTags::Config RobotContainer::camRoboRioConfig() {
	AprilTags::Config config;
	config.cameraName = "camRoboRio";
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -13.199602_in, -6.375_in, 18.100568_in, {0_deg, -20.0_deg, 180.0_deg} };}; //o -160
	return config;
}

/*AprilTags::Config RobotContainer::camTurretConfig(Turret* turret) {
	AprilTags::Config config;
	config.cameraName = "camTurret";
	config.cameraToRobotSupplier = [=] {
		auto transform = turret->GetRobotToCameraTransform();
		frc::SmartDashboard::PutNumber("RobotToCameraX", transform.X().value());
		frc::SmartDashboard::PutNumber("RobotToCameraY", transform.Y().value());
		return transform;
	};
	return config;
}*/