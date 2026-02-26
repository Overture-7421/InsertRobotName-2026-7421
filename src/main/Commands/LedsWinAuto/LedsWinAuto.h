// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/Commands.h>
#include <OvertureLib/Subsystems/LedsManager/LedsManager.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h>

frc2::CommandPtr LedsWinAuto(LedsManager* leds);