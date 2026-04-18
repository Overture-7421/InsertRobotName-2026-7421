// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Hood/Hood.h"
#include "Subsystems/Chassis/Chassis.h"
#include "Manager/LaunchModeManager/LaunchModeManager.h"
#include "OvertureLib/Gamepads/OverXboxController/OverXboxController.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Processor/Processor.h"
#include "Commands/VisionAlignCmd/VisionAlignCmd.h"
#include "Commands/EjectCommand/EjectCommand.h"

frc2::CommandPtr LaunchCommand(Shooter* shooter, Hood* hood, Chassis* chassis, LaunchModeManager* launchModeManager, std::function<double()> multiSupplier, OverXboxController* driver, Intake* intake, Processor* processor);