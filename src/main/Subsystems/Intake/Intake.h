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
  Intake();
  
  void setRollersVoltage(units::volt_t targetVoltage);
  void setIntakeAngle(units::degree_t targetAngle);
  bool reachedTarget();
  void UpdateTelemetry();

  void Periodic() override;

 private:
 OverTalonFX intakeMotor {IntakeConstants::intakeMotorConfig(), RobotConstants::RIO};
 OverTalonFX intakeSecondMotor {IntakeConstants::intakeSecondMotorConfig(), RobotConstants::RIO};
 OverTalonFX rollersMotor {IntakeConstants::rollersMotorConfig(), RobotConstants::RIO};
 OverCANCoder intakeCANCoder {IntakeConstants::intakeCanCoderConfig(), RobotConstants::RIO};
 OverCANCoder intakeSecondCANCoder {IntakeConstants::intakeSecondCanCoderConfig(), RobotConstants::RIO};

 ctre::phoenix6::controls::MotionMagicVoltage intakeVoltage {0_tr};
 ctre::phoenix6::controls::VoltageOut rollersVoltage {0_V};
};
