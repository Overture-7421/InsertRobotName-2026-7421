// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchCommand.h"

frc2::CommandPtr LaunchCommand(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver, Intake* intake, Processor* processor) {
	return frc2::cmd::Sequence(
		VisionAlignCmd(shooter, hood, chassis, launchModeManager, multiSupplier, driver).ToPtr(),

		frc2::cmd::Parallel(
			EjectCommand(intake, shooter, processor).ToPtr(),
			VisionAlignCmd(shooter, hood, chassis, launchModeManager, multiSupplier, driver, true).ToPtr()
		)

	);
}
