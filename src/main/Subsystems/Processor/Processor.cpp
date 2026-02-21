// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Processor.h"

Processor::Processor() = default;

void Processor::setSpindexerPasserVoltage(ProcessorValues processorValues){
    spindexerMotor.SetControl(spindexerVoltage.WithOutput(processorValues.spindexer).WithEnableFOC(true));
    passerMotor.SetControl(passerVoltage.WithOutput(processorValues.passer).WithEnableFOC(true));
}

void Processor::setOnlySpindexer(units::volt_t voltage){
    spindexerMotor.SetControl(spindexerVoltage.WithOutput(voltage).WithEnableFOC(true));
}

frc2::CommandPtr Processor::setProcessorCmd(ProcessorValues processorValues){
    return this->RunOnce([this, processorValues] {
    this->setSpindexerPasserVoltage(processorValues);
    });
}

frc2::CommandPtr Processor::setOnlySpindexerCmd(units::volt_t voltage){
    return this->RunOnce([this, voltage] {
    this->setOnlySpindexer(voltage);
    });
}

bool Processor::isFuelCharged() {
    return canRange.GetIsDetected().GetValue();
}

// Auto-preload API
void Processor::setAutoPreloadEnabled(bool enabled) {
    m_autoPreloadEnabled.store(enabled);
    if (!enabled) {
        // parar inmediatamente si se desactiva
        setSpindexerPasserVoltage(ProcessorConstants::StopProcessor);
    }
}

bool Processor::isAutoPreloadEnabled() const {
    return m_autoPreloadEnabled.load();
}

void Processor::notifyIntakeRunning(bool running) {
    m_intakeRequested.store(running);
}

bool Processor::isIntakeNotifiedRunning() const {
    return m_intakeRequested.load();
}

// This method will be called once per scheduler run
void Processor::Periodic() {
    const bool wantPreload = (m_autoPreloadEnabled.load() || m_intakeRequested.load());
    const bool hasBall = isFuelCharged();

    // opcional: telemetría para depurar
    frc::SmartDashboard::PutBoolean("Processor/WantPreload", wantPreload);
    frc::SmartDashboard::PutBoolean("Processor/HasBall", hasBall);

    if (wantPreload && !hasBall) {
        // aplicar voltajes para mover una bola hacia el CANrange
        setSpindexerPasserVoltage(ProcessorConstants::PreloadProcessor);
    } else {
        // parar si no queremos o ya hay bola
        setSpindexerPasserVoltage(ProcessorConstants::StopProcessor);
    }


}
