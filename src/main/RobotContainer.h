// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "OvertureLib/Gamepads/OverXboxController/OverXboxController.h"
#include "OvertureLib/Gamepads/OverConsole/OverConsole.h"
#include "Subsystems/Chassis/Chassis.h"
#include "Commands/DriveCommand/DriveCommand.h"
#include "Commands/ResetHeading/ResetHeading.h"
#include "Constants.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Processor/Processor.h"

#include "Subsystems/Turret/Turret.h"
#include "Subsystems/Shooter/Shooter.h"
#include <atomic>
#include "Commands/LaunchCommand/LaunchCommand.h"
#include <OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h>
#include "Commands/SwallowCommand/SwallowCommand.h"
#include "Commands/CloseCommand/CloseCommand.h"
#include "Commands/StopCommand/StopCommand.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  Chassis chassis;

  void UpdateTelemetry();


 private:
  OverXboxController driver{ 0, 0.05, 0.2 };
	OverXboxController oprtr{ 1, 0.20, 0.2 };
	OverConsole console{ 2 };
	OverXboxController test{ 3, 0.20, 0.2 };

  #ifndef __FRC_ROBORIO__
	frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);
#else
	frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);
	//frc::AprilTagFieldLayout tagLayout{ "/home/lvuser/deploy/tag_layout/7421-field.json" };
#endif 


  Shooter shooter;
  Turret turret{&chassis};
  Intake intake;
  Processor processor;

  LaunchModeManager launchModeManager;


  // The robot's subsystems are defined here...
	frc::SendableChooser<frc2::Command*> autoChooser;

  // necessary robot controllers and subsystems

  void ConfigureBindings();
  void ConfigDriverBindings();
  void ConfigOperatorBindings();

  static AprilTags::Config railCameraRight();
	static AprilTags::Config climberCameraLeft();
	static AprilTags::Config climberCameraRight();
	static AprilTags::Config railCameraLeft();

	AprilTags railCamRight{ &tagLayout, &chassis, railCameraRight() };
	AprilTags climberCamLeft{ &tagLayout, &chassis, climberCameraLeft() };
	AprilTags climberCamRight{ &tagLayout, &chassis, climberCameraRight() };
	AprilTags railCamLeft{ &tagLayout, &chassis, railCameraLeft() };

	std::atomic<const frc::Translation2d*> selectedTarget{ &LaunchConstants::HubPose };


};