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
                                         ShooterConstants::CruiseJerk);

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
    units::turns_per_second_t shooterError = targetVelocity - shooterLeftUpMotor.GetVelocity().GetValue();
    return units::math::abs(shooterError) < ShooterConstants::RangeOfError;
}

const ShooterState& Shooter::getState() {
    return state;
}

void Shooter::Hold() {
    this->shouldHold = true;
}

void Shooter::Release() {
    this->shouldHold = false;
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
    frc::SmartDashboard::PutBoolean("Shooter/isShooterAtVelocity", isShooterAtVelocity());
    frc::SmartDashboard::PutNumber("Shooter/averagekV", averagekV);
    frc::SmartDashboard::PutNumber("Shooter/PIDSlot", currentPIDSlot);
    frc::SmartDashboard::PutNumber("Shooter/State", (int) state);

}

void Shooter::Periodic() {
    // bool isAtTarget = isShooterAtVelocity();
    // units::turns_per_second_t currentVel = shooterLeftUpMotor.GetVelocity().GetValue();
    // units::volt_t currentVoltage = shooterLeftUpMotor.GetMotorVoltage().GetValue();

    // switch (state)
    // {
    // case ShooterState::WindUp: {
    //     currentPIDSlot = 0;
    //     if(isAtTarget && units::math::abs(targetVelocity) >= 0.001_tps) {
    //         lastTimeOnTarget = frc::Timer::GetFPGATimestamp();
    //         kVEstimator.reset();

    //         state = ShooterState::PreparingToHold;
    //     }   

    //     break;
    // }
    // case ShooterState::PreparingToHold: {
    //     currentPIDSlot = 0;
    //     units::second_t currentTime = frc::Timer::GetFPGATimestamp();
    //     if (!isAtTarget) {
    //         state = ShooterState::WindUp;
    //     }


    //     if (units::math::abs(currentVel) >= 0.001_tps){
    //         kVEstimator.emplace_front(units::math::abs(currentVoltage / currentVel).value());
    //     }

    //     if(currentTime - lastTimeOnTarget > 0.1_s && kVEstimator.size() == 20) {
    //         averagekV = 0.0;
    //         for(auto& kV : kVEstimator) {
    //             averagekV += kV;
    //         }

    //         averagekV /= (double) kVEstimator.size();

    //         ctre::phoenix6::configs::TalonFXConfiguration shooterLeftCTREConfig = shooterLeftUpMotor.getCTREConfig();
    //         ctre::phoenix6::configs::Slot1Configs slot1Config {};
          
    //         slot1Config.kV = averagekV;

    //         shooterLeftCTREConfig.Slot1 = slot1Config;
    //         shooterLeftUpMotor.GetConfigurator().Apply(shooterLeftCTREConfig);
    //         state = ShooterState::Holding;
    //     }

    //     break;
    // }
    // case ShooterState::Holding: {
    //     currentPIDSlot = 1;

    //     if (!isAtTarget && !shouldHold) {
    //         state = ShooterState::WindUp;
    //     }

    //     break;
    // }
    // default:
    //     break;
    // }

    shooterLeftUpMotor.SetControl(shooterVoltageRequest.WithVelocity(targetVelocity).WithEnableFOC(true).WithSlot(currentPIDSlot));
}
