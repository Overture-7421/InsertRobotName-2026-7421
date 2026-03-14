// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/Sensors/OverCANCoder/OverCANCoder.h>
#include <units/velocity.h>

struct ClimberConstants{
    constexpr static const units::degree_t  ClimberOpen = 45.0_deg;
    constexpr static const units::degree_t  ClimberStow = 0.0_deg;
    constexpr static const units::degree_t  ClimberInitial = 0.0_deg;

    constexpr static const double ClimberSensorToMechanism = 20.0;  
    constexpr static const units::meter_t WinchRadius = 0.08_m;
    constexpr static const units::meters_per_second_t ClimberCruiseVelocity = 1_mps;
    constexpr static const units::meters_per_second_squared_t ClimberCruiseAcceleration = 0.5_mps_sq;
    constexpr static const units::meter_t ClimberRangeError = 0.01_m;
    
    constexpr static OverTalonFXConfig ClimberMotorConfig() {
        OverTalonFXConfig climberMotorConfig;
        climberMotorConfig.MotorId = 24;
        climberMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
        climberMotorConfig.Inverted = true;
        climberMotorConfig.useFOC = true;
        climberMotorConfig.PIDConfigs.WithKP(400.0);

        climberMotorConfig.ClosedLoopRampRate = 0.01_s;
        climberMotorConfig.CurrentLimit = 30_A;
        climberMotorConfig.OpenLoopRampRate = 0.05_s;
        climberMotorConfig.StatorCurrentLimit = 120_A;
        climberMotorConfig.TriggerThreshold = 40_A;
        climberMotorConfig.TriggerThresholdTime = 0.5_s;
    
        return climberMotorConfig;
    }
};
