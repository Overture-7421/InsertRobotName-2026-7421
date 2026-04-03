// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Processor.h"

Processor::Processor(){}

void Processor::setProcessorVoltages(processorValues voltages){
    indexerRightMotor.SetControl(spindexerVoltage.WithOutput(voltages.indexerVoltage).WithEnableFOC(true));
    passerUpMotor.SetControl(passerVoltage.WithOutput(voltages.passerVoltage).WithEnableFOC(true));
}


frc2::CommandPtr Processor::setProcessorCmd(processorValues voltages){
     return frc2::FunctionalCommand(
        [this, voltages] () {
            setProcessorVoltages(voltages);
        },

        [] () {
        },

        [this] (bool interrupted){},

        [this] {
            return true;
        }, {this}
    ).ToPtr();
}

bool Processor::isPasserActive(){
    return units::math::abs(passerUpMotor.GetMotorVoltage().GetValue()) > 0.0_V;
}

// This method will be called once per scheduler run
void Processor::Periodic() {

}
