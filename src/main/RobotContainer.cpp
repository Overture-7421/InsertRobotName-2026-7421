// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer(){
	//Sujto a cambio
	pathplanner::NamedCommands::registerCommand("SwallowCommand", std::move(intake.setIntakeCmd(IntakeConstants::IntakeOpen)));
	pathplanner::NamedCommands::registerCommand("IntakeSustain", std::move(intake.setIntakeCmd(IntakeConstants::IntakeSustain)));
	pathplanner::NamedCommands::registerCommand("EjectCommand", std::move(LaunchCommand(&shooter, &hood, &chassis, &intake, &processor, &launchModeManager, [this] {return launchShooterMulti;}, &driver).ToPtr()).WithTimeout(5.0_s));

	pathplanner::NamedCommands::registerCommand("AfterEject", std::move(frc2::cmd::Parallel(processor.setProcessorCmd(ProcessorConstants::StopProcessor), hood.setHoodAngleCommand(HoodConstants::Close))));



	autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	ConfigDriverBindings();
	ConfigOperatorBindings();
	ConfigConsoleBindings();
	ConfigTestBindings();
}

void RobotContainer::ConfigDriverBindings() {
	//Button A Disparo Manual pegado al climb zone

	chassis.SetDefaultCommand(DriveCommand(&chassis, &driver, &processor).ToPtr());
	driver.Back().OnTrue(ResetHeading(&chassis));

	driver.LeftBumper().WhileTrue(intake.setIntakeCmd(IntakeConstants::IntakeOpen));
	driver.LeftBumper().OnFalse(intake.setIntakeCmd(IntakeConstants::IntakeSustain));

	driver.RightBumper().WhileTrue(LaunchCommand(&shooter, &hood, &chassis, &intake, &processor, &launchModeManager, [this] {return launchShooterMulti;}, &driver).ToPtr());
	driver.RightBumper().OnFalse(frc2::cmd::Parallel(processor.setProcessorCmd(ProcessorConstants::StopProcessor), hood.setHoodAngleCommand(HoodConstants::Close)));


	// driver.LeftBumper().ToggleOnTrue(TabulateCommand(&shooter, &chassis, &turret, &launchModeManager).ToPtr());



	(isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", { 0, 255, 0 }).ToPtr().IgnoringDisable(true)); //Our turn
	(isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", { 0, 255, 0 }, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Over

	(!isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", { 107, 53, 170 }).ToPtr().IgnoringDisable(true)); //Inactive
	(!isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", { 107, 53, 170 }, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Our Turn
}

void RobotContainer::ConfigOperatorBindings() {

	oprtr.A().OnTrue(frc2::cmd::RunOnce([this] {
		launchModeManager.setLaunchMode(LaunchModes::Hub);
	}));

	oprtr.Y().OnTrue(frc2::cmd::RunOnce([this] {
		launchModeManager.setLaunchMode(LaunchModes::Pass);
	}));

	oprtr.B().WhileTrue(CloseCommand(&intake, &processor));
	oprtr.B().OnFalse(CloseCommand(&intake, &processor));

	oprtr.POVUp().OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti += 0.03;
	}));

	oprtr.POVDown().OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti -= 0.03;
	}));
}

void RobotContainer::ConfigConsoleBindings() {

	console.Button(2).OnTrue(frc2::cmd::RunOnce([this] {
		launchModeManager.setLaunchMode(LaunchModes::Hub);
	}));

	console.Button(1).OnTrue(frc2::cmd::RunOnce([this] {
		launchModeManager.setLaunchMode(LaunchModes::Pass);
	}));

	console.Button(3).WhileTrue(CloseCommand(&intake, &processor));
	console.Button(3).OnFalse(CloseCommand(&intake, &processor));

	console.Button(5).OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti += 0.03;
	}));

	console.Button(11).OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti -= 0.03;
	}));


}

void RobotContainer::ConfigTestBindings() {
	//TEST

	//Shooter
	// test.A().WhileTrue(shooter.setShooterVelocityCmd(35_tps));
	// test.A().OnFalse(shooter.setShooterVelocityCmd(25_tps));

	//Hood
	// test.B().WhileTrue(hood.setHoodAngleCommand(24.0_deg));
	// test.B().OnFalse(hood.setHoodAngleCommand(2.0_deg));

	//Intake
	// test.A().WhileTrue(intake.setIntakeCharacterization(0.30_m, 7_V));
	// test.A().OnFalse(intake.setIntakeCharacterization(0.10_m, 0_V));

	// test.B().WhileTrue(intake.setIntakeCharacterization(0.30_m, 7_V));
	// test.B().OnFalse(intake.setIntakeSlowModeCmd(intakeValues{0_V, 0.10_m}));

	//Processor
	// test.A().WhileTrue(processor.setProcessorCmd(ProcessorValues{6_V, 0_V}));
	// test.A().OnFalse(processor.setProcessorCmd(ProcessorConstants::StopProcessor));

	//Intake Swallow
	// test.LeftBumper().WhileTrue(intake.setIntakeCmd(IntakeConstants::IntakeOpen));
	// test.LeftBumper().OnFalse(intake.setIntakeCmd(IntakeConstants::IntakeSustain));

	//Launch Test
	// test.RightBumper().WhileTrue(frc2::cmd::Parallel(shooter.setShooterVelocityCmd(30_tps), hood.setHoodAngleCommand(10.0_deg), processor.setProcessorCmd(ProcessorConstants::Eject)));
	// test.RightBumper().OnFalse(frc2::cmd::Parallel(shooter.setShooterVelocityCmd(0_tps), hood.setHoodAngleCommand(HoodConstants::Close), processor.setProcessorCmd(ProcessorConstants::StopProcessor)));


}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}

void RobotContainer::UpdateTelemetry() {
	chassis.shuffleboardPeriodic();
	shooter.UpdateTelemetry();
	intake.UpdateTelemetry();
	hood.UpdateTelemetry();

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
