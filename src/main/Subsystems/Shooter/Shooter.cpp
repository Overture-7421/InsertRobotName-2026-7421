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
    this->targetVelocity = velocity;
}

units::turns_per_second_t Shooter::getShooterVelocity(){
    return shooterLeftUpMotor.GetVelocity().GetValue();
}

bool Shooter::isShooterAtVelocity(){
    units::turns_per_second_t shooterError = units::turns_per_second_t(shooterLeftDownMotor.GetClosedLoopError().GetValue());
    return units::math::abs(shooterError) < ShooterConstants::RangeOfError;
}

const ShooterState& Shooter::getState() {
    return state;
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
    frc::SmartDashboard::PutNumber("Shooter/ErrorVelocity", shooterLeftUpMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Shooter/PID/TargetVelocity", targetVelocity.value());
    frc::SmartDashboard::PutNumber("Shooter/PID/HoldingVelocity", holdingTargetVelocity.value());
    frc::SmartDashboard::PutBoolean("Shooter/isShooterAtVelocity", isShooterAtVelocity());
    frc::SmartDashboard::PutNumber("Shooter/State", (int) state);

}

void Shooter::Periodic() {

    if ((units::math::abs(holdingTargetVelocity - targetVelocity) > 0.1_tps)){
        state = ShooterState::WindUp;
    }

    switch (state)
    {
    case ShooterState::WindUp: {
        shooterLeftUpMotor.SetControl(shooterVoltageRequest.WithVelocity(targetVelocity).WithEnableFOC(false));
        if(isShooterAtVelocity()) {
            lastTimeOnTarget = frc::Timer::GetFPGATimestamp();
            holdingTargetVelocity = targetVelocity;
            state = ShooterState::PreparingToHold;
        }

        break;
    }
    case ShooterState::PreparingToHold: {
        units::second_t currentTime = frc::Timer::GetFPGATimestamp();

        if (!isShooterAtVelocity()) {
            state = ShooterState::WindUp;
            break;
        }

        if(currentTime - lastTimeOnTarget > 0.15_s) {
            state = ShooterState::Holding;
            holdingVoltage = shooterLeftUpMotor.GetMotorVoltage().GetValue();
        }

        break;
    }
    case ShooterState::Holding: {
        shooterLeftUpMotor.SetControl(shooterHoldingVoltageRequest.WithOutput(holdingVoltage).WithEnableFOC(false));
        break;
    }
    default:
        break;
    }
}
