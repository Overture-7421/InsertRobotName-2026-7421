// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
    shooterRightMotor.setFollow(shooterLeftMotor.GetDeviceID(), false);
    shooterLeftMotor.setSensorToMechanism(ShooterConstants::ShooterSensorToMechanism);
    shooterLeftMotor.configureMotionMagic(ShooterConstants::ShooterCruiseVelocity,
                                         ShooterConstants::ShooterCruiseAcceleration,
                                         0.0_tr_per_s_cu);

    hoodMotor.setRotorToSensorRatio(ShooterConstants::HoodRotorToSensor);
    hoodMotor.setSensorToMechanism(ShooterConstants::HoodSensorToMechanism);
    // hoodMotor.SetPosition(hoodCANCoder.GetAbsolutePosition().GetValue());
    hoodMotor.setFusedCANCoder(ShooterConstants::HoodCANCoderId);
    hoodMotor.configureMotionMagic(ShooterConstants::HoodCruiseVelocity,
                                  ShooterConstants::HoodCruiseAcceleration,
                                  0.0_tr_per_s_cu);

                                  
    ctre::phoenix6::configs::TalonFXConfiguration shooterLeftCTREConfig = shooterLeftMotor.getCTREConfig();
    shooterLeftCTREConfig.Feedback.VelocityFilterTimeConstant = 0.1_s;
    shooterLeftMotor.GetConfigurator().Apply(shooterLeftCTREConfig);
}

void Shooter::setObjectiveVelocity(units::turns_per_second_t velocity){
    shooterLeftMotor.SetControl(shooterVoltageRequest.WithVelocity(velocity).WithEnableFOC(false));
}

units::turns_per_second_t Shooter::getShooterVelocity(){
    return shooterLeftMotor.GetVelocity().GetValue();
}

bool Shooter::isShooterAtVelocity(units::turns_per_second_t targetVelocity){ //Creo que esta bien
    units::turns_per_second_t shooterError = targetVelocity - shooterLeftMotor.GetVelocity().GetValue();
    return units::math::abs(shooterError) < 2.0_tps;
}

frc2::CommandPtr Shooter::setShooterVelocityCommand(units::turns_per_second_t velocity){
    return frc2::cmd::RunOnce(
        [this, velocity] {
            setObjectiveVelocity(velocity);
        }
    );
}

void Shooter::setHoodAngle(units::degree_t angle){
    hoodMotor.SetControl(hoodVoltageRequest.WithPosition(angle).WithEnableFOC(false));

}

units::degree_t Shooter::getHoodAngle(){
    return hoodMotor.GetPosition().GetValue();
}

bool Shooter::isHoodAtAngle(units::degree_t targetAngle){
    units::degree_t hoodError = targetAngle - hoodMotor.GetPosition().GetValue();
    return units::math::abs(hoodError) < 2.0_deg;
}

frc2::CommandPtr Shooter::setHoodAngleCommand(units::degree_t angle){
    return frc2::cmd::RunOnce(
        [this, angle] {
            setHoodAngle(angle);
        }
    );
}

void Shooter::UpdateTelemetry(){
    frc::SmartDashboard::PutNumber("Shooter/Shooter/ActualVelocity", shooterLeftMotor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("Shooter/Hood/ActualAngle", hoodMotor.GetPosition().GetValue().value() * 360.0);
    frc::SmartDashboard::PutNumber("Shooter/Hood/EncoderAngle", hoodCANCoder.GetAbsolutePosition().GetValue().value()* 360.0);

    double targetVelocity = shooterLeftMotor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("Shooter/Shooter/ErrorVelocity", shooterLeftMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Shooter/Shooter/TargetVelocity", targetVelocity);
    frc::SmartDashboard::PutBoolean("Shooter/Shooter/isShooterAtVelocity", isShooterAtVelocity(units::turns_per_second_t(targetVelocity)));


    double targetAngle = hoodMotor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("Shooter/Hood/ErrorAngle", hoodMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Shooter/Hood/TargetAngle", targetAngle);
    frc::SmartDashboard::PutBoolean("Shooter/Hood/isHoodAtAngle", isHoodAtAngle(units::degree_t(targetAngle)));
}

void Shooter::Periodic() {

}
