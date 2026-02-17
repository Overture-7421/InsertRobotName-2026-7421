// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwallowCommand.h"

frc2::CommandPtr SwallowCommand(Intake* intake, Processor* processor){
    return frc2::cmd::Parallel(
        intake->setIntakePosition(IntakeConstants::IntakeOpen),
        processor->setOnlySpindexerCmd(ProcessorConstants::AccomodateSpindexer)
    );
}