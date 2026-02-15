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
  frc2::CommandPtr setShooterVelocityCommand(units::turns_per_second_t velocity);

  void setHoodAngle(units::degree_t angle);
  units::degree_t getHoodAngle();
  bool isHoodAtAngle(units::degree_t targetAngle);
  frc2::CommandPtr setHoodAngleCommand(units::degree_t angle);

  void UpdateTelemetry();


  void Periodic() override;

 private:

 OverTalonFX shooterLeftMotor{ShooterConstants::ShooterLeftConfig(), robotConstants::rio};
 OverTalonFX shooterRightMotor{ShooterConstants::ShooterRightConfig(), robotConstants::rio};

 OverTalonFX hoodMotor{ShooterConstants::HoodConfig(), robotConstants::rio};
 OverCANCoder hoodCANCoder{ShooterConstants::HoodCANConfig(), robotConstants::rio};

  ctre::phoenix6::controls::MotionMagicVelocityVoltage shooterVoltageRequest{0.0_tps};
  ctre::phoenix6::controls::MotionMagicVoltage hoodVoltageRequest{0.0_tr};
  
};
