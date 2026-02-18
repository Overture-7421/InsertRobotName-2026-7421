// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/OverTalonFX/Config.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"
#include "OvertureLib/Sensors/OverCANCoder/Config.h"
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>



namespace ClimberConstants {

    constexpr static const double ClimberRotorToSensorRatio = 62.5;
    constexpr static const double ClimberCANCoderId = 17;

    constexpr static const units::turns_per_second_t ClimberMMCruiseVelocity =  4.0_tps ;
    constexpr static const units::turns_per_second_squared_t ClimberMMAcceleration = 42_tr_per_s_sq;
    constexpr static const units::turns_per_second_cubed_t ClimberMMJerk = 0.0_tr_per_s_cu;
    
OverTalonFXConfig ClimberConstants(){
    OverTalonFXConfig climberConstants;
    
    climberConstants.MotorId = 1; // Example motor ID, change as needed


    climberConstants.NeutralMode = ControllerNeutralMode::Brake;
    climberConstants.Inverted = false; // Set to true if the motor is inverted
    climberConstants.useFOC = false; // Set to true if using Field Oriented Control


    climberConstants.PIDConfigs.WithKP(0.00).WithKI(0.0).WithKD(0.0);
    


    climberConstants.CurrentLimit = 30_A; // Example current limit, adjust as needed
    climberConstants.StatorCurrentLimit = 40_A; // Example stator current limit
    climberConstants.TriggerThreshold = 50_A; // Example trigger threshold
    climberConstants.TriggerThresholdTime = 0.5_s; // Example trigger threshold time
    climberConstants.ClosedLoopRampRate = 0.1_s; // Example closed loop ramp rate
    climberConstants.OpenLoopRampRate = 0.1_s; // Example open loop ramp rate

    return climberConstants;
}

CanCoderConfig getCANCodeConfig(){

    CanCoderConfig climberCANCoder;
    
    climberCANCoder.CanCoderId = -1;
    climberCANCoder.Offset = 0_deg;
    climberCANCoder.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue:: CounterClockwise_Positive;

    return climberCANCoder;
}
}