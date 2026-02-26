// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CloseCommand.h"

frc2::CommandPtr CloseCommand(Intake* intake, Processor* processor){
    return frc2::cmd::Parallel(
        intake->setIntakePosition(IntakeConstants::IntakeClose),
        processor->setProcessorCmd(ProcessorConstants::StopProcessor)
    );
}
