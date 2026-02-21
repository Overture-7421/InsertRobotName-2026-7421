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


class Processor : public frc2::SubsystemBase {
 public:
  Processor();

  void setSpindexerPasserVoltage(ProcessorValues processorValues);
  void setOnlySpindexer(units::volt_t voltage);

  frc2::CommandPtr setProcessorCmd(ProcessorValues processorValues);
  frc2::CommandPtr setOnlySpindexerCmd(units::volt_t voltage);

  bool isFuelCharged();

  // Auto‑preload control (automatic; LaunchCommand will disable while launching)
  void setAutoPreloadEnabled(bool enabled);
  bool isAutoPreloadEnabled() const;

  // Notify intake running (optional: used if intake wants processor to preload while intake runs)
  void notifyIntakeRunning(bool running);
  bool isIntakeNotifiedRunning() const;
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

    OverTalonFX spindexerMotor{ProcessorConstants::SpindexerConfig(), robotConstants::rio};
    OverTalonFX passerMotor{ProcessorConstants::PasserConfig(), robotConstants::rio};

    ctre::phoenix6::controls::VoltageOut spindexerVoltage{0_V};
    ctre::phoenix6::controls::VoltageOut passerVoltage{0_V};

    ctre::phoenix6::hardware::CANrange canRange {29, robotConstants::rio};

    // flags
    std::atomic<bool> m_autoPreloadEnabled{true};   // preloading activo por defecto
    std::atomic<bool> m_intakeRequested{false};     // notificado por intake

};
