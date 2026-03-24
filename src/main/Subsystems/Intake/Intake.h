// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "IntakeConstants.h"
#include "Constants.h"
#include <frc2/command/Commands.h>

class Intake : public frc2::SubsystemBase {
 public:
  units::degree_t transformCentimetersToDegrees(units::centimeter_t distance);
  units::centimeter_t transformDegreesToCentimeters(units::degree_t angle);
  Intake();
  
  
  void setRollersVoltage(units::volt_t targetVoltage);
  void setIntakeDistance(units::centimeter_t targetDistance);
  bool intakeReached(units::centimeter_t targetDistance);
  
  


  frc2::CommandPtr setIntakeCmd(intakeValues targetPos);
  frc2::CommandPtr setIntakeCharacterization(units::centimeter_t distance, units::volt_t voltage);
  frc2::CommandPtr setRollersCmd(units::volt_t targetVoltage);
  frc2::CommandPtr setPivotCmd(units::centimeter_t targetDistance);

  void UpdateTelemetry();

  void Periodic() override;

 private:
 OverTalonFX rightMotor {IntakeConstants::RightMotorConfig(), robotConstants::rio};
 OverTalonFX leftMotor {IntakeConstants::LeftMotorConfig(), robotConstants::rio};
 OverTalonFX rollersMotor {IntakeConstants::rollersMotorConfig(), robotConstants::rio};
 OverCANCoder rightCANCoder {IntakeConstants::RightCanCoderConfig(), robotConstants::rio};
 OverCANCoder leftCANCoder {IntakeConstants::LeftCanCoderConfig(), robotConstants::rio};

 ctre::phoenix6::controls::MotionMagicVoltage intakeVoltage {0_tr};
 ctre::phoenix6::controls::VoltageOut rollersVoltage {0_V};
};
