// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "IntakeConstants.h"
#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  
  void setRollersVoltage(units::volt_t targetVoltage);
  void setIntakeAngle(units::degree_t targetAngle);
  bool intakeReached(units::degree_t targetAngle);

  frc2::CommandPtr setIntakePosition(intakeValues targetPos);
  frc2::CommandPtr setRollersVoltageCommand(units::volt_t targetVoltage);

  void Periodic() override;

 private:
 OverTalonFX intakeMotor {IntakeConstants::intakeMotorConfig(), robotConstants::rio};
 OverTalonFX rollersMotor {IntakeConstants::rollersMotorConfig(), robotConstants::rio};
 OverCANCoder intakeCANCoder {IntakeConstants::intakeCanCoderConfig(), robotConstants::rio};

 ctre::phoenix6::controls::MotionMagicVoltage intakeVoltage {0_tr};
 ctre::phoenix6::controls::VoltageOut rollersVoltage {0_V};
};
