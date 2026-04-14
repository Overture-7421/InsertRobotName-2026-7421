// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Hood/Hood.h"
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Processor/Processor.h"
#include "OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h"
#include "pathplanner/lib/util/FlippingUtil.h"
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "VisionAlignConstants.h"
#include "Manager/LaunchModeManager/LaunchModeManager.h"
#include "OvertureLib/Gamepads/OverXboxController/OverXboxController.h"
#include "PassTargetSwitcher.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class VisionAlignCmd
	: public frc2::CommandHelper<frc2::Command, VisionAlignCmd> {
public:
	/* You should consider using the more terse Command factories API instead
	 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
	 */
	VisionAlignCmd(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver, bool shouldEnd = false);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;


	Shooter* shooter = nullptr;
	Hood* hood = nullptr;
	Chassis* chassis = nullptr;
	OverXboxController* driver = nullptr;

	LaunchModeManager* launchModeManager = nullptr;


	frc::ProfiledPIDController<units::radian> headingController{
		// PID constants: 
				4.75, 0.0, 0.15, {10_rad_per_s, 14_rad_per_s_sq * 1.00} //Constraints max velocity, max acceleration
	};
	HeadingSpeedsHelper headingSpeedsHelper;

	std::function<double()> multiSupplier;

	TargetingWhileMoving targetWhileMoving{ //Tiempo de Vuelo desde que sale la pieza hasta que llega al objetivo
	  {
		{1.45_m, 1.08_s},
		{1.95_m, 1.09_s},
		{2.45_m, 1.15_s},
		{2.95_m, 1.16_s},
		{3.45_m, 1.17_s},
		{3.95_m, 1.21_s},
		{4.45_m, 1.23_s},
	  }, 0.01_s
	};

	units::degree_t chassisError;
	bool shouldEnd = false;

	nt::StructPublisher<frc::Translation2d> targetPublisher =
		nt::NetworkTableInstance::GetDefault().GetStructTopic < frc::Translation2d
		>("SmartDashboard/MovingTarget").Publish();

	PassTargetSwitcher passTargetSwitcher {LaunchConstants::LeftPass, LaunchConstants::RightPass, 0.2_m};

};
