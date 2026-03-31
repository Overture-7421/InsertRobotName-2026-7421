// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
    shooterLeftDownMotor.setFollow(shooterLeftUpMotor.GetDeviceID(), false);
    shooterRightUpMotor.setFollow(shooterLeftUpMotor.GetDeviceID(), true);
    shooterRightDownMotor.setFollow(shooterLeftUpMotor.GetDeviceID(), true);

    shooterLeftUpMotor.setSensorToMechanism(ShooterConstants::SensorToMechanism);
    shooterLeftUpMotor.configureMotionMagic(ShooterConstants::CruiseVelocity,
                                         ShooterConstants::CruiseAcceleration,
                                         0.0_tr_per_s_cu);

    ctre::phoenix6::configs::TalonFXConfiguration shooterLeftCTREConfig = shooterLeftUpMotor.getCTREConfig();
    shooterLeftCTREConfig.Feedback.VelocityFilterTimeConstant = 0.1_s;
    shooterLeftUpMotor.GetConfigurator().Apply(shooterLeftCTREConfig);
}

void Shooter::setObjectiveVelocity(units::turns_per_second_t velocity){
    shooterLeftUpMotor.SetControl(shooterVoltageRequest.WithVelocity(velocity).WithEnableFOC(false));
}

units::turns_per_second_t Shooter::getShooterVelocity(){
    return shooterLeftUpMotor.GetVelocity().GetValue();
}

bool Shooter::isShooterAtVelocity(units::turns_per_second_t targetVelocity){
    units::turns_per_second_t shooterError = targetVelocity - shooterLeftUpMotor.GetVelocity().GetValue();
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
    frc::SmartDashboard::PutNumber("Shooter/PID/ActualVelocity", shooterLeftUpMotor.GetVelocity().GetValue().value());

    double targetVelocity = shooterLeftUpMotor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("Shooter/ErrorVelocity", shooterLeftUpMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Shooter/PID/TargetVelocity", targetVelocity);
    frc::SmartDashboard::PutBoolean("Shooter/isShooterAtVelocity", isShooterAtVelocity(units::turns_per_second_t(targetVelocity)));

}

void Shooter::Periodic() {

}
