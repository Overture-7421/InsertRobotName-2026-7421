// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "EjectCommand.h"

frc2::CommandPtr EjectCommand(Intake* intake, Processor* processor){
    return frc2::cmd::Parallel(
        intake->setIntakeCmd(IntakeConstants::IntakeClose),
        processor->setProcessorCmd(ProcessorConstants::Eject)
    ).BeforeStarting(frc2::cmd::RunOnce([intake]{return intake->setIntakeLowerSpeed();})).FinallyDo([intake]{return intake->setIntakeNormalSpeed();});
}
