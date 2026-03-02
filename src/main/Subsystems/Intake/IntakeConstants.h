// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/Sensors/OverCANCoder/OverCANCoder.h>


struct intakeValues{
    units::volt_t rollers;
    units::degree_t intake;
};

struct IntakeConstants{
    
    constexpr static const intakeValues IntakeOpen {5_V, -151.0_deg}; //Poner todas las posiciones del intake, nada esta puesto bien.
    constexpr static const intakeValues IntakeGiver {5_V, -130.0_deg};
    constexpr static const intakeValues IntakeSustain {0_V, -151.0_deg};
    constexpr static const intakeValues IntakeClose {0_V, -8_deg};
    constexpr static const intakeValues IntakeInitial {0_V, 0_deg};


    constexpr static const double intakeRotorToSensor = 50.0;  
    constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 2_tps;
    constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 6_tr_per_s_sq;
    constexpr static const units::degree_t IntakeRangeError = 1_deg;
    
    constexpr static OverTalonFXConfig intakeMotorConfig() {
        OverTalonFXConfig intakeMotorConfig;
        intakeMotorConfig.MotorId = 14;
        intakeMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeMotorConfig.Inverted = true;
        intakeMotorConfig.useFOC = true;
        intakeMotorConfig.PIDConfigs.WithKP(55.0);

        intakeMotorConfig.ClosedLoopRampRate = 0.05_s;
        intakeMotorConfig.CurrentLimit = 30_A;
        intakeMotorConfig.OpenLoopRampRate = 0.0_s;
        intakeMotorConfig.StatorCurrentLimit = 120_A;
        intakeMotorConfig.TriggerThreshold = 40_A;
    
        return intakeMotorConfig;
    }
    
    constexpr static OverTalonFXConfig rollersMotorConfig() {
        OverTalonFXConfig rollersMotorConfig;
        rollersMotorConfig.MotorId = 16;
        rollersMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
        rollersMotorConfig.Inverted = false;
        rollersMotorConfig.useFOC = true;
    
        rollersMotorConfig.ClosedLoopRampRate = 0.0_s;
        rollersMotorConfig.CurrentLimit = 30_A;
        rollersMotorConfig.OpenLoopRampRate = 0.05_s;
        rollersMotorConfig.StatorCurrentLimit = 120_A;
        rollersMotorConfig.TriggerThreshold = 40_A;
        rollersMotorConfig.TriggerThresholdTime = 0.5_s;
    
        return rollersMotorConfig;
    }

    constexpr static CanCoderConfig intakeCanCoderConfig(){

        // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)
        
        CanCoderConfig intakeCanCoderConfig;
        intakeCanCoderConfig.CanCoderId = 15;
        intakeCanCoderConfig.Offset = -0.492919921875_tr;
        intakeCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;

        return intakeCanCoderConfig;
    }
};
