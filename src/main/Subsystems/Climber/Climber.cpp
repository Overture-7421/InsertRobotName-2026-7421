// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"

// Grabber.cpp


ClimberSubsystem::ClimberSubsystem(){
    climberMotor.setRotorToSensorRatio(ClimberConstants::ClimberRotorToSensorRatio);
    climberMotor.setFusedCANCoder(ClimberConstants::ClimberCANCoderId);
    climberMotor.configureMotionMagic(ClimberConstants::ClimberMMCruiseVelocity, ClimberConstants::ClimberMMAcceleration, ClimberConstants::ClimberMMJerk);
};


void ClimberSubsystem::moveClimber(units::degree_t target){
    climberMotor.SetControl(climberPositionRequest.WithPosition(target).WithEnableFOC(true));
};

bool ClimberSubsystem::isClimberFinished(units::degree_t target){
    return units::math::abs(climberMotor.GetPosition().GetValue() - target)<1.0_deg;
}

units::degree_t ClimberSubsystem::climberGetPosition(){
     return climberMotor.GetPosition().GetValue();
}



frc2::CommandPtr ClimberSubsystem::SetPosition(units::degree_t target) {
    return frc2::FunctionalCommand(
        // Init
        [this, target] { moveClimber(target); },
        // onExecute
        [this] { },
        // onEnd
        [this] (bool interrupted) { },
        // isFinished
        [this, target] { return isClimberFinished(target); }
    ).ToPtr();
}