// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Processor.h"

Processor::Processor(){
    indexer2Motor.setFollow(indexer1Motor.GetDeviceID(), false);
    passer2Motor.setFollow(passer1Motor.GetDeviceID(), false);
}

void Processor::setProcessorVoltages(ProcessorValues processorValues){
    indexer1Motor.SetControl(spindexerVoltage.WithOutput(processorValues.spindexer).WithEnableFOC(true));
    passer1Motor.SetControl(passerVoltage.WithOutput(processorValues.passer).WithEnableFOC(true));
}

void Processor::setOnlySpindexer(units::volt_t voltage){
    indexer1Motor.SetControl(spindexerVoltage.WithOutput(voltage).WithEnableFOC(true));
}

frc2::CommandPtr Processor::setProcessorCmd(ProcessorValues processorValues){
     return frc2::FunctionalCommand(
        [this, processorValues] () {
            setProcessorVoltages(processorValues);

        },

        [] () {
        },

        [this] (bool interrupted){},

        [this] {
            return true;
        }, {this}
    ).ToPtr();
}

frc2::CommandPtr Processor::setOnlySpindexerCmd(units::volt_t voltage){
    return frc2::cmd::RunOnce([this, voltage] {
    this->setOnlySpindexer(voltage);
    }, {this});
}

bool Processor::isPasserActive(){
    return units::math::abs(passer1Motor.GetMotorVoltage().GetValue()) > 0.0_V;
}

bool Processor::isFuelCharged() {
    return canRange.GetIsDetected().GetValue();
}

// This method will be called once per scheduler run
void Processor::Periodic() {

}
