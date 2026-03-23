// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Hood/Hood.h"
#include "Subsystems/Chassis/Chassis.h"
#include "OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h"
#include "pathplanner/lib/util/FlippingUtil.h"
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "LaunchConstants.h"
#include "Manager/LaunchModeManager/LaunchModeManager.h"
#include "Subsystems/Processor/Processor.h"
#include "OvertureLib/Gamepads/OverXboxController/OverXboxController.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LaunchCommand
	: public frc2::CommandHelper<frc2::Command, LaunchCommand> {
public:
	/* You should consider using the more terse Command factories API instead
	 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
	 */
	LaunchCommand(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;


	Shooter* shooter = nullptr;
	Hood* hood = nullptr;
	Chassis* chassis = nullptr;

	OverXboxController* driver = nullptr;

	// std::function<frc::Translation2d()> targetSupplier;

	LaunchModeManager* launchModeManager = nullptr;

	std::function<double()> multiSupplier;

	TargetingWhileMoving targetWhileMoving{ //Tiempo de Vuelo desde que sale la pieza hasta que llega al objetivo
	  {
		{1.50_m, 0.94_s},
		{2.00_m, 1.08_s},
		{2.50_m, 1.12_s},
		{3.00_m, 1.16_s},
		{3.50_m, 1.17_s},
		{4.00_m, 1.18_s},
		{4.50_m, 1.23_s},
		{5.00_m, 1.24_s}
	  }, 0.01_s
	};

	nt::StructPublisher<frc::Translation2d> targetPublisher =
		nt::NetworkTableInstance::GetDefault().GetStructTopic < frc::Translation2d
		>("SmartDashboard/MovingTarget").Publish();


};
