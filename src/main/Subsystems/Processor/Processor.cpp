// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Processor.h"

Processor::Processor(){
    passerUpMotor.setSensorToMechanism(ProcessorConstants::SensorToMechanism);
    passerUpMotor.configureMotionMagic(ProcessorConstants::CruiseVelocity,
                                          ProcessorConstants::CruiseAcceleration,
                                          0.0_tr_per_s_cu);

    ctre::phoenix6::configs::TalonFXConfiguration passerUpCTREConfig = passerUpMotor.getCTREConfig();
    passerUpCTREConfig.Feedback.VelocityFilterTimeConstant = 0.1_s;
    passerUpMotor.GetConfigurator().Apply(passerUpCTREConfig);
}

void Processor::setProcessorVoltages(units::volt_t indexerVoltage, units::turns_per_second_t passerVelocity){
    indexerRightMotor.SetControl(spindexerVoltage.WithOutput(indexerVoltage).WithEnableFOC(true));
    passerUpMotor.SetControl(passerVoltage.WithVelocity(passerVelocity).WithEnableFOC(true));
}

void Processor::setOnlySpindexer(units::volt_t voltage){
    indexerRightMotor.SetControl(spindexerVoltage.WithOutput(voltage).WithEnableFOC(true));
}

void Processor::setOnlyPasserVelocity(units::turns_per_second_t velocity){
    passerUpMotor.SetControl(passerVoltage.WithVelocity(velocity).WithEnableFOC(true));
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
    return units::math::abs(passerUpMotor.GetMotorVoltage().GetValue()) > 0.0_V;
}

bool Processor::isFuelCharged() {
    return canRange.GetIsDetected().GetValue();
}

bool Processor::isPasserAtVelocity(){
    units::turns_per_second_t passerError = units::turns_per_second_t(passerUpMotor.GetClosedLoopError().GetValue());
    return units::math::abs(passerError) < ProcessorConstants::RangeOfError;
}

void Processor::UpdateTelemetry(){
    frc::SmartDashboard::PutNumber("Processor/PID/ActualVelocity", passerUpMotor.GetVelocity().GetValue().value());

    double targetVelocity = passerUpMotor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("Processor/ErrorVelocity", passerUpMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Processor/PID/TargetVelocity", targetVelocity);
    frc::SmartDashboard::PutBoolean("Processor/isPasserAtVelocity", isPasserAtVelocity());

}

// This method will be called once per scheduler run
void Processor::Periodic() {

}
