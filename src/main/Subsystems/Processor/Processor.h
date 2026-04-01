// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Subsystems/Processor/ProcessorConstants.h"
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include "Constants.h"
#include <frc2/command/FunctionalCommand.h>
#include <ctre/phoenix6/CANrange.hpp>
#include <atomic>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>


class Processor : public frc2::SubsystemBase {
 public:
  Processor();

  void setProcessorVoltages(units::volt_t indexerVoltage, units::turns_per_second_t passerVelocity);
  void setOnlySpindexer(units::volt_t voltage);
  void setOnlyPasserVelocity(units::turns_per_second_t velocity);

  frc2::CommandPtr setProcessorCmd(units::volt_t indexerVoltage, units::turns_per_second_t passerVelocity);
  frc2::CommandPtr setOnlySpindexerCmd(units::volt_t voltage);
  bool isPasserActive();

  frc2::CommandPtr setPasserVelocityCmd(units::turns_per_second_t velocity);

  bool isFuelCharged();

  void Periodic() override;

 private:

    OverTalonFX indexerRightMotor{ProcessorConstants::IndexerRightConfig(), robotConstants::rio};
    OverTalonFX passerDownMotor{ProcessorConstants::PasserDownConfig(), robotConstants::rio};

    ctre::phoenix6::controls::VoltageOut spindexerVoltage{0_V};
  ctre::phoenix6::controls::MotionMagicVelocityVoltage passerVoltage{0.0_tps};


    ctre::phoenix6::hardware::CANrange canRange {29, robotConstants::rio};

};
