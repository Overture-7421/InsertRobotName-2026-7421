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
  bool climberReached(units::degree_t targetAngle);

  frc2::CommandPtr setClimberPosition(climberValues targetPos);
 
  void Periodic() override;

 private:
 OverTalonFX climberMotor {climberConstants::climberMotorConfig(), robotConstants::rio};
 OverCANCoder climberCANCoder {climberConstants::climberCanCoderConfig(), robotConstants::rio};

 ctre::phoenix6::controls::MotionMagicVoltage climberVoltage {0_tr};
 };
