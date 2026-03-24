// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
    shooter2Motor.setFollow(shooter1Motor.GetDeviceID(), false);
    shooter3Motor.setFollow(shooter1Motor.GetDeviceID(), false);
    shooter4Motor.setFollow(shooter1Motor.GetDeviceID(), false);

    shooter1Motor.setSensorToMechanism(ShooterConstants::SensorToMechanism);
    shooter1Motor.configureMotionMagic(ShooterConstants::CruiseVelocity,
                                         ShooterConstants::CruiseAcceleration,
                                         0.0_tr_per_s_cu);

    ctre::phoenix6::configs::TalonFXConfiguration shooterLeftCTREConfig = shooter1Motor.getCTREConfig();
    shooterLeftCTREConfig.Feedback.VelocityFilterTimeConstant = 0.1_s;
    shooter1Motor.GetConfigurator().Apply(shooterLeftCTREConfig);
}

void Shooter::setObjectiveVelocity(units::turns_per_second_t velocity){
    shooter1Motor.SetControl(shooterVoltageRequest.WithVelocity(velocity).WithEnableFOC(false));
}

units::turns_per_second_t Shooter::getShooterVelocity(){
    return shooter1Motor.GetVelocity().GetValue();
}

bool Shooter::isShooterAtVelocity(units::turns_per_second_t targetVelocity){
    units::turns_per_second_t shooterError = targetVelocity - shooter1Motor.GetVelocity().GetValue();
    return units::math::abs(shooterError) < ShooterConstants::RangeOfError;
}

frc2::CommandPtr Shooter::setShooterVelocityCmd(units::turns_per_second_t velocity){
    return frc2::cmd::RunOnce(
        [this, velocity] {
            setObjectiveVelocity(velocity);
        }, {this}
    );
}

void Shooter::UpdateTelemetry(){
    frc::SmartDashboard::PutNumber("Shooter/Shooter/ActualVelocity", shooter1Motor.GetVelocity().GetValue().value());

    double targetVelocity = shooter1Motor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("Shooter/Shooter/ErrorVelocity", shooter1Motor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Shooter/Shooter/TargetVelocity", targetVelocity);
    frc::SmartDashboard::PutBoolean("Shooter/Shooter/isShooterAtVelocity", isShooterAtVelocity(units::turns_per_second_t(targetVelocity)));

}

void Shooter::Periodic() {

}
