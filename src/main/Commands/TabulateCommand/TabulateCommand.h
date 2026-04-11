// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <Subsystems/Shooter/Shooter.h>
#include <Subsystems/Hood/Hood.h>
#include <Subsystems/Chassis/Chassis.h>
#include "OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h"
#include "pathplanner/lib/util/FlippingUtil.h"
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "Manager/LaunchModeManager/LaunchModeManager.h"
#include "Commands/VisionAlignCmd/VisionAlignConstants.h"
#include "Commands/VisionAlignCmd/PassTargetSwitcher.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TabulateCommand
	: public frc2::CommandHelper<frc2::Command, TabulateCommand> {
public:
	/* You should consider using the more terse Command factories API instead
	 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
	 */
	TabulateCommand(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

	Shooter* shooter;
	Hood* hood;
	Chassis* chassis;
	
	// std::function<frc::Translation2d()> targetSupplier;
	LaunchModeManager* launchModeManager = nullptr;


	PassTargetSwitcher passTargetSwitcher {LaunchConstants::LeftPass, LaunchConstants::RightPass, 0.2_m};

	frc::ProfiledPIDController<units::radian> headingController{
		// PID constants: 
				4.75, 0.0, 0.15, {13_rad_per_s, 18_rad_per_s_sq * 2} //Constraints max velocity, max acceleration
	};
	HeadingSpeedsHelper headingSpeedsHelper;


};
