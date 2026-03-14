// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"
#include "ClimberConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber() {
    climberMotor.setSensorToMechanism(ClimberConstants::ClimberSensorToMechanism) ;
    climberMotor.configureMotionMagic(ClimberConstants::ClimberCruiseVelocity, ClimberConstants::ClimberCruiseAcceleration, 0.0_tr_per_s_cu);
}

units::meter_t Climber::getHeight(){
    return climberMotor.GetPosition() * ClimberConstants::ClimberSensorToMechanism;
}

bool Climber::reachedTarget(units::degree_t targetAngle){
    return (climberMotor.GetClosedLoopError().GetValue() < ClimberConstants::ClimberRangeError);
}

void Climber::setClimberAngle(units::degree_t targetAngle){
    climberMotor.SetControl(climberVoltage.WithPosition(targetAngle).WithEnableFOC(true));
}


frc2::CommandPtr Climber::setClimberCmd(units::degree_t targetPos){
    return frc2::FunctionalCommand(
        [this, targetPos] () {
            setClimberAngle(targetPos);
        },

        [this] () {
            
        },

        [this] (bool interrupted){},

        [this, targetPos] {
            return climberReached(targetPos);
        }
    ).ToPtr();
}

void Climber::UpdateTelemetry(){
    frc::SmartDashboard::PutNumber("Climber/Current", climberMotor.GetPosition().GetValue().value() * 360.0);
}

// This method will be called once per scheduler run
void Climber::Periodic() {
}
