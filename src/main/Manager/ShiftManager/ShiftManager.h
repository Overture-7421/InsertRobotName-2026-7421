// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

struct HubState {
  bool isActive = false;
  bool isTransitioning = false;
};

class ShiftManager : public frc2::SubsystemBase {
 public:
  ShiftManager();

  // 2. The function that calculates the current game state
  HubState GetHubState() const;

  void Periodic() override;

 private:
};