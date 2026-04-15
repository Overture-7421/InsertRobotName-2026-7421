// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Subsystems/Processor/ProcessorConstants.h"
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include "Constants.h"
#include <frc2/command/FunctionalCommand.h>
#include <atomic>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>


class Processor : public frc2::SubsystemBase {
 public:
  Processor();

  void setProcessorVoltages(processorValues voltages);

  frc2::CommandPtr setProcessorCmd(processorValues voltages);
  
  bool isPasserActive();

  void Periodic() override;

 private:

    OverTalonFX indexerRightMotor{ProcessorConstants::IndexerRightConfig(), robotConstants::rio};
    OverTalonFX passerUpMotor{ProcessorConstants::PasserUpConfig(), robotConstants::rio};
    OverTalonFX passerDownMotor{ProcessorConstants::PasserDownConfig(), robotConstants::rio};

    ctre::phoenix6::controls::VoltageOut spindexerVoltage{0_V};
    ctre::phoenix6::controls::VoltageOut passerVoltage{0_V};

};
