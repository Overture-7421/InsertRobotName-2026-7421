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
#include "Constants.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Chassis/Chassis.h"

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

 private:
	frc::SendableChooser<frc2::Command*> autoChooser;

  // necessary robot controllers and subsystems
  OverXboxController driver{ 0, 0.05, 0.2 };

  void ConfigureBindings();
  void ConfigDriverBindings();

  Intake intake;
  Chassis chassis;


};