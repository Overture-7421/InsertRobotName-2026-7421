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
  units::turn_t transformMetersToTurns(units::meter_t distance);
  units::meter_t transformTurnsToMeters(units::turn_t angle);
  Intake();
  
  
  void setRollersVoltage(units::volt_t targetVoltage);
  void setIntakeDistance(units::meter_t targetDistance);
  bool intakeReached(units::meter_t targetDistance);

  
    void setIntakeLowerSpeed();
    void setIntakeNormalSpeed();

  frc2::CommandPtr setIntakeCmd(intakeValues targetPos);
  frc2::CommandPtr setRollersCmd(units::volt_t targetVoltage);
  frc2::CommandPtr setPivotCmd(units::meter_t targetDistance);

  frc2::CommandPtr setIntakeCharacterization(units::meter_t distance, units::volt_t voltage);

  void UpdateTelemetry();

  void Periodic() override;

 private:
 OverTalonFX sliderRightMotor {IntakeConstants::sliderRightMotorConfig(), robotConstants::rio};
 OverTalonFX sliderLeftMotor {IntakeConstants::sliderLeftMotorConfig(), robotConstants::rio};
 OverTalonFX rollersRightMotor {IntakeConstants::rollersRightMotorConfig(), robotConstants::rio};
 OverTalonFX rollersLeftMotor {IntakeConstants::rollersLeftMotorConfig(), robotConstants::rio};
 OverCANCoder sliderRightCanCoder {IntakeConstants::SliderCanCoderConfig(), robotConstants::rio};

 ctre::phoenix6::controls::MotionMagicVoltage intakeVoltage {0_tr};
 ctre::phoenix6::controls::VoltageOut rollersVoltage {0_V};
};
