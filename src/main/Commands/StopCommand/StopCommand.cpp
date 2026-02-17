// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StopCommand.h"

frc2::CommandPtr StopCommand(Intake* intake, Processor* processor){
    return frc2::cmd::Parallel(
        intake->setIntakePosition(IntakeConstants::IntakeSustain),
        processor->setProcessorCmd(ProcessorConstants::StopProcessor)
    );
}