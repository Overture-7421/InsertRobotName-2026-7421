// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
	//Sujto a cambio
	pathplanner::NamedCommands::registerCommand("SwallowCommand", std::move(intake.setIntakeCmd(IntakeConstants::IntakeAuto)));
	pathplanner::NamedCommands::registerCommand("IntakeSustain", std::move(intake.setIntakeCmd(IntakeConstants::IntakeSustain)));
	pathplanner::NamedCommands::registerCommand("VisionAlignCmd", std::move(VisionAlignCmd(&shooter, &hood, &chassis, &launchModeManager, [this] {return launchShooterMulti;}, &driver, true).ToPtr()).WithTimeout(3.0_s));
	pathplanner::NamedCommands::registerCommand("VisionAlignCmdInfinite", std::move(VisionAlignCmd(&shooter, &hood, &chassis, &launchModeManager, [this] {return launchShooterMulti;}, &driver).ToPtr()));

	pathplanner::NamedCommands::registerCommand("EjectCommand", std::move(EjectCommand(&intake, &shooter, &processor).ToPtr()).WithTimeout(3.2_s));

	pathplanner::NamedCommands::registerCommand("AfterEject", std::move(frc2::cmd::Parallel(processor.setProcessorCmd(ProcessorConstants::Stop), hood.setHoodAngleCommand(HoodConstants::Close), shooter.setShooterVelocityCmd(ShooterConstants::SustainVelocity))));



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

	driver.LeftTrigger().WhileTrue(intake.setIntakeCmd(IntakeConstants::IntakeOpen));
	driver.LeftTrigger().OnFalse(intake.setIntakeCmd(IntakeConstants::IntakeSustain));

	driver.RightTrigger().WhileTrue(VisionAlignCmd(&shooter, &hood, &chassis, &launchModeManager, [this] {return launchShooterMulti;}, &driver).ToPtr());
	driver.RightTrigger().OnFalse(frc2::cmd::Parallel(processor.setProcessorCmd(ProcessorConstants::Stop), hood.setHoodAngleCommand(HoodConstants::Close), shooter.setShooterVelocityCmd(ShooterConstants::SustainVelocity)));


	//For the Pit
	driver.Y().WhileTrue(frc2::cmd::Parallel(shooter.setShooterVelocityCmd(10_tps), processor.setProcessorCmd(ProcessorConstants::Spit)));
	driver.Y().OnFalse(frc2::cmd::Parallel(shooter.setShooterVelocityCmd(0_tps), processor.setProcessorCmd(ProcessorConstants::Stop)));


	//Tabulate
	// driver.A().ToggleOnTrue(TabulateCommand(&shooter, &hood, &chassis, &launchModeManager).ToPtr());

	// driver.B().WhileTrue(frc2::cmd::Sequence(processor.setProcessorCmd(ProcessorConstants::Eject),
	//  	intake.setRollersCmd(IntakeConstants::IntakeOpen.rollers),
	// 	frc2::cmd::Wait(0.7_s),
	// 	intake.setRollersCmd(IntakeConstants::IntakeClose.rollers),
	// 	intake.setIntakeSlowModeCmd(IntakeConstants::IntakeClose)
	// 	).BeforeStarting(frc2::cmd::RunOnce([this] {shooter.Hold();})).FinallyDo([this] {shooter.Release();}));
	// driver.B().OnFalse(frc2::cmd::Parallel(processor.setProcessorCmd(ProcessorConstants::Stop),
	// 										intake.setSliderCmd(IntakeConstants::IntakeOpen.intake)
	// 										));



	//Leds
	// (isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", { 0, 255, 0 }).ToPtr().IgnoringDisable(true)); //Our turn
	// (isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", { 0, 255, 0 }, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Over

	// (!isHubActive && !isTransitioning).WhileTrue(StaticEffect(&leds, "all", { 107, 53, 170 }).ToPtr().IgnoringDisable(true)); //Inactive
	// (!isHubActive && isTransitioning).WhileTrue(BlinkEffect(&leds, "all", { 107, 53, 170 }, 0.2_s).ToPtr().IgnoringDisable(true)); //Almost Our Turn

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
	//The buttons of the console counts from 1 to 12, not from 0(which is the normal)

	console.Button(2).OnTrue(frc2::cmd::RunOnce([this] {
		launchModeManager.setLaunchMode(LaunchModes::Hub);
	}));

	console.Button(4).OnTrue(frc2::cmd::RunOnce([this] {
		launchModeManager.setLaunchMode(LaunchModes::Pass);
	}));

	console.Button(5).WhileTrue(CloseCommand(&intake, &processor));
	console.Button(5).OnFalse(CloseCommand(&intake, &processor));

	console.Button(1).OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti += 0.03;
	}));

	console.Button(11).OnTrue(frc2::cmd::RunOnce([this] {
		this->launchShooterMulti -= 0.03;
	}));

	console.Button(6).WhileTrue(EjectCommand(&intake, &shooter, &processor).ToPtr().OnlyWhile([this] {return driver.GetHID().GetLeftTriggerAxis() < 0.1;}));
	//Ver si tengo que agregar un OnFalse
	//Y ponerlo tambien para el oprtr

	console.Button(3).WhileTrue(frc2::cmd::Parallel(intake.setRollersCmd(IntakeConstants::RollersReverse), processor.setProcessorCmd(ProcessorConstants::Reverse)));
	console.Button(3).OnFalse(frc2::cmd::Parallel(intake.setRollersCmd(IntakeConstants::RollersStop), processor.setProcessorCmd(ProcessorConstants::Stop)));


}

void RobotContainer::ConfigTestBindings() {
	//TEST

	//Shooter
	test.B().WhileTrue(shooter.setShooterVelocityCmd(36_tps).BeforeStarting(frc2::cmd::RunOnce([this] {shooter.Hold();})).FinallyDo([this] {shooter.Release();}));
	test.B().OnFalse(shooter.setShooterVelocityCmd(20_tps).BeforeStarting(frc2::cmd::RunOnce([this] {shooter.Hold();})).FinallyDo([this] {shooter.Release();}));

	//Hood
	test.A().WhileTrue(hood.setHoodAngleCommand(26.5_deg));
	test.A().OnFalse(hood.setHoodAngleCommand(2.0_deg));

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

	// //Launch Test
	// test.RightBumper().WhileTrue(frc2::cmd::Parallel(shooter.setShooterVelocityCmd(30_tps), hood.setHoodAngleCommand(25.0_deg), processor.setProcessorCmd(ProcessorConstants::Eject), intake.setIntakeSlowModeCmd(IntakeConstants::IntakeClose)));
	// test.RightBumper().OnFalse(frc2::cmd::Parallel(shooter.setShooterVelocityCmd(0_tps), hood.setHoodAngleCommand(HoodConstants::Close), processor.setProcessorCmd(ProcessorConstants::Stop)));


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

AprilTags::Config RobotContainer::camRightConfig() {
	AprilTags::Config config;
	config.backend = VisionBackend::PhotonVision;
	config.cameraName = "camRight";
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -0.348978_in, -12.730269_in, 13.438423_in, {0_deg, -25_deg, -90.0_deg} };};
	return config;
}

AprilTags::Config RobotContainer::camLeftConfig() {
	AprilTags::Config config;
	config.backend = VisionBackend::PhotonVision;
	config.cameraName = "camLeft";
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -0.348978_in , 12.730269_in, 13.438423_in, {0_deg, -25_deg, 90.0_deg} };};
	return config;
}

AprilTags::Config RobotContainer::limelightUpConfig() {
	AprilTags::Config config;
	config.backend = VisionBackend::Limelight;
	config.cameraName = "limelight-up";
	config.limelightMode = LimelightMode::MegaTag2;
	config.multiTagStdDevs = wpi::array<double, 3>  { 0.5, 0.5, 0.5 };
	config.yawCorrectionThreshold = 10_deg;
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -0.345435_m, 0.047625_m, 0.444663_m, {0_deg, -15_deg, 180.0_deg} };};
	return config;
}

AprilTags::Config RobotContainer::limelightDownConfig() {
	AprilTags::Config config;
	config.backend = VisionBackend::Limelight;
	config.cameraName = "limelight-down";
	config.limelightMode = LimelightMode::MegaTag2;
	config.multiTagStdDevs = wpi::array<double, 3>  { 0.5, 0.5, 0.5 };
	config.yawCorrectionThreshold = 10_deg;
	config.cameraToRobotSupplier = [] {return frc::Transform3d{ -0.334501_m, 0.047625_m, 0.320287_m, {0_deg, -40.0_deg, 180.0_deg} };};
	return config;
}
