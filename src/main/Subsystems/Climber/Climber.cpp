// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"
#include "ClimberConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber() {
    climberMotor.setRotorToSensorRatio(climberConstants::climberRotorToSensor) ;
    climberMotor.setFusedCANCoder(climberConstants::climberCanCoderConfig().CanCoderId);


    climberMotor.configureMotionMagic(climberConstants::ClimberCruiseVelocity, climberConstants::ClimberCruiseAcceleration, 0.0_tr_per_s_cu);
}


bool Climber::climberReached(units::degree_t targetAngle){
    units::degree_t climberError = targetAngle - climberMotor.GetPosition().GetValue();
    return (units::math::abs(climberError) < climberConstants::ClimberRangeError);
}

void Climber::setClimberAngle(units::degree_t targetAngle){
    frc::SmartDashboard::PutNumber("Climber Target Position:", targetAngle.value());
    climberMotor.SetControl(climberVoltage.WithPosition(targetAngle).WithEnableFOC(true));
}


frc2::CommandPtr Climber::setClimberPosition(climberValues targetPos){
    return frc2::FunctionalCommand(
        [this, targetPos] () {
            setClimberAngle(targetPos.climber);
        },

        [this] () {
            
        },

        [this] (bool interrupted){},

        [this, targetPos] {
            return (climberReached(targetPos.climber));
        }
    ).ToPtr();
}


// This method will be called once per scheduler run
void Climber::Periodic() {
    frc::SmartDashboard::PutNumber("Climber Position", climberMotor.GetPosition().GetValue().value() * 360.0);
}
