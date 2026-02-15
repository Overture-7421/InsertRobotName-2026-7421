// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LaunchModeManager.h"

LaunchModeManager::LaunchModeManager() = default;

LaunchModes LaunchModeManager::getLaunchMode(){
    return launchMode;
}

void LaunchModeManager::setLaunchMode(LaunchModes desiredLaunchMode){
    this->launchMode = desiredLaunchMode;
}

frc2::CommandPtr LaunchModeManager::setLaunchModeCmd(LaunchModes desiredLaunchMode){
    return frc2::cmd::RunOnce([this, desiredLaunchMode]{
        this->launchMode = desiredLaunchMode;
    });
}