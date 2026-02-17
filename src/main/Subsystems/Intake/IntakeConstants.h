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
    
    constexpr static const intakeValues IntakeOpen {6_V, 30.0_deg};
    constexpr static const intakeValues IntakeSustain {0_V, 30.0_deg};
    constexpr static const intakeValues IntakeClose {0_V, -30_deg};
    constexpr static const intakeValues IntakeInitial {0_V, 0_deg};


    constexpr static const double intakeRotorToSensor = 62.5;  
    constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 95_tps;
    constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 280_tr_per_s_sq;
    constexpr static const units::degree_t IntakeRangeError = 1_deg;
    constexpr static const units::turn_t IntakeCANCoderOffset = 0_tr;
    
    constexpr static OverTalonFXConfig intakeMotorConfig() {
        
        // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)
        OverTalonFXConfig intakeMotorConfig;
        intakeMotorConfig.MotorId = 14;
        intakeMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeMotorConfig.Inverted = false;
        intakeMotorConfig.useFOC = true;
        intakeMotorConfig.PIDConfigs.WithKP(20).WithKV(2);

        intakeMotorConfig.ClosedLoopRampRate = 0.0_s;
        intakeMotorConfig.CurrentLimit = 30_A;
        intakeMotorConfig.OpenLoopRampRate = 0.05_s;
        intakeMotorConfig.StatorCurrentLimit = 120_A;
        intakeMotorConfig.TriggerThreshold = 40_A;
        intakeMotorConfig.TriggerThresholdTime = 0.5_s;
    
        return intakeMotorConfig;
    }
    
    constexpr static OverTalonFXConfig rollersMotorConfig() {

        // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)

        OverTalonFXConfig rollersMotorConfig;
        rollersMotorConfig.MotorId = 16;
        rollersMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
        rollersMotorConfig.Inverted = true;
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
        intakeCanCoderConfig.Offset = IntakeCANCoderOffset;
        intakeCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;

        return intakeCanCoderConfig;
    }
};
