// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Processor.h"

Processor::Processor(){
}

void Processor::setProcessorVoltages(units::volt_t indexerVoltage, units::turns_per_second_t passerVelocity){
    indexerRightMotor.SetControl(spindexerVoltage.WithOutput(indexerVoltage).WithEnableFOC(true));
    passerDownMotor.SetControl(passerVoltage.WithVelocity(passerVelocity).WithEnableFOC(true));
}

void Processor::setOnlySpindexer(units::volt_t voltage){
    indexerRightMotor.SetControl(spindexerVoltage.WithOutput(voltage).WithEnableFOC(true));
}

void Processor::setOnlyPasserVelocity(units::turns_per_second_t velocity){
    passerDownMotor.SetControl(passerVoltage.WithVelocity(velocity).WithEnableFOC(true));
}


frc2::CommandPtr Processor::setProcessorCmd(units::volt_t indexerVoltage, units::turns_per_second_t passerVelocity){
     return frc2::FunctionalCommand(
        [this, indexerVoltage, passerVelocity] () {
            setProcessorVoltages(indexerVoltage, passerVelocity);

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

frc2::CommandPtr Processor::setPasserVelocityCmd(units::turns_per_second_t velocity){
    return frc2::cmd::RunOnce(
        [this, velocity] {
            setOnlyPasserVelocity(velocity);
        }, {this}
    );
}

bool Processor::isPasserActive(){
    return units::math::abs(passerDownMotor.GetMotorVoltage().GetValue()) > 0.0_V;
}

bool Processor::isFuelCharged() {
    return canRange.GetIsDetected().GetValue();
}

// This method will be called once per scheduler run
void Processor::Periodic() {

}
