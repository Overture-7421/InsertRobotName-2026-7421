// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Enums/LaunchModes.h"
#include "frc2/command/Commands.h"


class LaunchModeManager {
 public:
  LaunchModeManager();

  LaunchModes getLaunchMode();
  void setLaunchMode(LaunchModes desiredLaunchMode);
  frc2::CommandPtr setLaunchModeCmd(LaunchModes desiredLaunchMode);

  LaunchModes launchMode = LaunchModes::Hub;
};
