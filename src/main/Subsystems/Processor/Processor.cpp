// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Processor.h"

Processor::Processor(){
    // indexerRightMotor.setFollow(indexerLeftMotor.GetDeviceID(), true);
    passerDownMotor.setFollow(passerUpMotor.GetDeviceID(), false);
}

void Processor::setProcessorVoltages(ProcessorValues processorValues){
    indexerRightMotor.SetControl(spindexerVoltage.WithOutput(processorValues.spindexer).WithEnableFOC(true));
    passerUpMotor.SetControl(passerVoltage.WithOutput(processorValues.passer).WithEnableFOC(true));
}

void Processor::setOnlySpindexer(units::volt_t voltage){
    indexerRightMotor.SetControl(spindexerVoltage.WithOutput(voltage).WithEnableFOC(true));
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
    return units::math::abs(passerUpMotor.GetMotorVoltage().GetValue()) > 0.0_V;
}

bool Processor::isFuelCharged() {
    return canRange.GetIsDetected().GetValue();
}

// This method will be called once per scheduler run
void Processor::Periodic() {

}
