// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ClimberConstants.h"
#include "Constants.h"

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  
  void setClimberAngle(units::degree_t targetAngle);
  bool reachedTarget();
  units::meter_t getHeight();

  void UpdateTelemetry(); 
  void Periodic() override;

 private:
 OverTalonFX climberMotor {ClimberConstants::ClimberMotorConfig(), RobotConstants::RIO};
 };
