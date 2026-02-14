// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
    shooterRightMotor.setFollow(shooterLeftMotor.GetDeviceID(), true);
    shooterLeftMotor.setSensorToMechanism(ShooterConstants::ShooterSensorToMechanism);
    shooterLeftMotor.configureMotionMagic(ShooterConstants::ShooterCruiseVelocity,
                                         ShooterConstants::ShooterCruiseAcceleration,
                                         0.0_tr_per_s_cu);

    hoodMotor.setSensorToMechanism(ShooterConstants::HoodSensorToMechanism);
    hoodMotor.setFusedCANCoder(ShooterConstants::HoodCANCoderId);
    hoodMotor.configureMotionMagic(ShooterConstants::HoodCruiseVelocity,
                                  ShooterConstants::HoodCruiseAcceleration,
                                  0.0_tr_per_s_cu);
}

void Shooter::setObjectiveVelocity(units::turns_per_second_t velocity){
    shooterLeftMotor.SetControl(shooterVoltageRequest.WithVelocity(velocity).WithEnableFOC(true));
    frc::SmartDashboard::PutNumber("Shooter/Shooter/TargetVelocity", velocity.value());
}

bool Shooter::isShooterAtVelocity(units::turns_per_second_t targetVelocity){ //Creo que esta bien
    units::turns_per_second_t shooterError = targetVelocity - shooterLeftMotor.GetVelocity().GetValue();
    return units::math::abs(shooterError) < 2_tps;
}

frc2::CommandPtr Shooter::setShooterVelocityCommand(units::turns_per_second_t velocity){
    return frc2::cmd::RunOnce(
        [this, velocity] {
            setObjectiveVelocity(velocity);
        }
    );
}

void Shooter::setHoodAngle(units::degree_t angle){
    hoodMotor.SetControl(hoodVoltageRequest.WithPosition(angle).WithEnableFOC(true));
    frc::SmartDashboard::PutNumber("Shooter/Hood/TargetAngle", angle.value());

}

bool Shooter::isHoodAtAngle(units::degree_t targetAngle){
    units::degree_t hoodError = targetAngle - hoodMotor.GetPosition().GetValue();
    return units::math::abs(hoodError) < 2_deg;
}

frc2::CommandPtr Shooter::setHoodAngleCommand(units::degree_t angle){
    return frc2::cmd::RunOnce(
        [this, angle] {
            setHoodAngle(angle);
        }
    );
}

void Shooter::Periodic() {

    frc::SmartDashboard::PutNumber("Shooter/Shooter/ActualVelocity", shooterLeftMotor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("Shooter/Hood/ActualAngle", hoodMotor.GetPosition().GetValue().value() * 360.0);

}
