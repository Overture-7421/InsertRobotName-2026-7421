// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/Sensors/OverCANCoder/OverCANCoder.h>


struct climberValues{
    units::degree_t climber;
};

struct climberConstants{
    
    constexpr static const climberValues ClimberOpen { 45.0_deg};
    constexpr static const climberValues ClimberStow {0_deg};
    constexpr static const climberValues ClimberInitial { 0_deg};


    constexpr static const double climberRotorToSensor = 62.5;  
    constexpr static const units::turns_per_second_t ClimberCruiseVelocity = 95_tps;
    constexpr static const units::turns_per_second_squared_t ClimberCruiseAcceleration = 95_tr_per_s_sq;
    constexpr static const units::degree_t ClimberRangeError = 2_deg;
    constexpr static const units::turn_t ClimberCANCoderOffset = 0.125_tr;
    
    constexpr static OverTalonFXConfig climberMotorConfig() {
        
        // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)
        OverTalonFXConfig climberMotorConfig;
        climberMotorConfig.MotorId = 14;
        climberMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
        climberMotorConfig.Inverted = true;
        climberMotorConfig.useFOC = true;
        climberMotorConfig.PIDConfigs.WithKP(27.5).WithKV(45);

        climberMotorConfig.ClosedLoopRampRate = 0.01_s;
        climberMotorConfig.CurrentLimit = 30_A;
        climberMotorConfig.OpenLoopRampRate = 0.05_s;
        climberMotorConfig.StatorCurrentLimit = 120_A;
        climberMotorConfig.TriggerThreshold = 40_A;
        climberMotorConfig.TriggerThresholdTime = 0.5_s;
    
        return climberMotorConfig;
    }
    
    
    constexpr static CanCoderConfig climberCanCoderConfig(){

        // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)
        
        CanCoderConfig climberCanCoderConfig;
        climberCanCoderConfig.CanCoderId = 15;
        climberCanCoderConfig.Offset = ClimberCANCoderOffset;
        climberCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;

        return climberCanCoderConfig;
    }
};
