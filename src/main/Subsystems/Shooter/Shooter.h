// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ShooterConstants.h"
#include "Constants.h"
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <units/math.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>


class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  void setObjectiveVelocity(units::turns_per_second_t velocity);
  units::turns_per_second_t getShooterVelocity();
  bool isShooterAtVelocity(units::turns_per_second_t targetVelocity);
  frc2::CommandPtr setShooterVelocityCmd(units::turns_per_second_t velocity);


  void UpdateTelemetry();


  void Periodic() override;

 private:

 OverTalonFX shooterLeftUpMotor{ShooterConstants::ShooterLeftUpConfig(), robotConstants::rio};
 OverTalonFX shooterLeftDownMotor{ShooterConstants::ShooterLeftDownConfig(), robotConstants::rio};
 OverTalonFX shooterRightUpMotor{ShooterConstants::ShooterRightUpConfig(), robotConstants::rio};
 OverTalonFX shooterRightDownMotor{ShooterConstants::ShooterRightDownConfig(), robotConstants::rio};

  ctre::phoenix6::controls::MotionMagicVelocityVoltage shooterVoltageRequest{0.0_tps};
  
};
