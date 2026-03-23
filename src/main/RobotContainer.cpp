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
	pathplanner::NamedCommands::registerCommand("ShooterStop", std::move(shooter.setShooterVelocityCmd(0_tps)));


	autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);

	ConfigureBindings();

	launchCommand = std::make_unique<LaunchCommand>(&shooter, &hood, &chassis, &launchModeManager, [this] {return launchShooterMulti;}, &driver);

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

	// driver.LeftBumper().WhileTrue(SwallowCommand(&intake));   Reahacer Logica
	// driver.LeftBumper().OnFalse(intake.setIntakeCmd(IntakeConstants::IntakeSustainWithoutRollers));


	driver.RightBumper().WhileTrue(processor.setProcessorCmd(ProcessorConstants::Eject));
	driver.RightBumper().OnFalse(processor.setProcessorCmd(ProcessorConstants::StopProcessor));

	// 	driver.Y().WhileTrue(CloseCommand(&intake, &processor));   Estaria padre tener este, nose si para el Operador, Igual Rehacer logica con Intake nuevo
	// 	driver.Y().OnFalse(CloseCommand(&intake, &processor));


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
	// test.A().WhileTrue(shooter.setShooterVelocityCommand(36_tps));
	// test.A().OnFalse(shooter.setShooterVelocityCommand(27_tps));

	//Hood
	// test.A().WhileTrue(shooter.setHoodAngleCommand(32.0_deg));
	// test.A().OnFalse(shooter.setHoodAngleCommand(3.0_deg));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}

void RobotContainer::UpdateTelemetry() {
	chassis.shuffleboardPeriodic();
	shooter.UpdateTelemetry();
	intake.UpdateTelemetry();

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
