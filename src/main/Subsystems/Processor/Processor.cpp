// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Processor.h"

Processor::Processor() = default;

void Processor::setSpindexerPasserVoltage(ProcessorValues processorValues){
    spindexerMotor.SetControl(spindexerVoltage.WithOutput(processorValues.spindexer).WithEnableFOC(true));
    passerMotor.SetControl(passerVoltage.WithOutput(processorValues.passer).WithEnableFOC(true));
}

frc2::CommandPtr Processor::setProcessorVoltage(ProcessorValues processorValues){
    return this->RunOnce([this, processorValues] {
    this->setSpindexerPasserVoltage(processorValues);
    });
}

// This method will be called once per scheduler run
void Processor::Periodic() {}
