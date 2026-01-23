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

struct intakeConstants{

  // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)
  
  constexpr static const intakeValues IntakeOpen {6_V, 180_deg};
  constexpr static const intakeValues IntakeStow {0_V, 0_deg};


  constexpr static const double intakeRotorToSensor = 1;
  constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 50_tps;
  constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 42_tr_per_s_sq;
  constexpr static const units::degree_t IntakeRangeError = 1_deg;
  constexpr static const units::turn_t IntakeCANCoderOffset = 1_tr;
  
  constexpr static OverTalonFXConfig intakeMotorConfig() {
    
    // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)

    OverTalonFXConfig intakeMotorConfig;
    intakeMotorConfig.MotorId = 14;
    intakeMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
    intakeMotorConfig.Inverted = true;
    intakeMotorConfig.useFOC = true;
    intakeMotorConfig.PIDConfigs.WithKP(0).WithKD(0).WithKV(0);
  
    intakeMotorConfig.ClosedLoopRampRate = 0.05_s;
    intakeMotorConfig.CurrentLimit = 1_A;
    intakeMotorConfig.OpenLoopRampRate = 0.05_s;
    intakeMotorConfig.StatorCurrentLimit = 1_A;
    intakeMotorConfig.TriggerThreshold = 1_A;
    intakeMotorConfig.TriggerThresholdTime = 0.05_s;
  
    return intakeMotorConfig;
  }
  
  constexpr static OverTalonFXConfig rollersMotorConfig() {

    // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)

    OverTalonFXConfig rollersMotorConfig;
    rollersMotorConfig.MotorId = 16;
    rollersMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
    rollersMotorConfig.Inverted = true;
    rollersMotorConfig.useFOC = true;
    rollersMotorConfig.PIDConfigs.WithKP(0).WithKD(0).WithKV(0);
  
    rollersMotorConfig.ClosedLoopRampRate = 0.05_s;
    rollersMotorConfig.CurrentLimit = 1_A;
    rollersMotorConfig.OpenLoopRampRate = 0.05_s;
    rollersMotorConfig.StatorCurrentLimit = 1_A;
    rollersMotorConfig.TriggerThreshold = 1_A;
    rollersMotorConfig.TriggerThresholdTime = 0.05_s;
  
    return rollersMotorConfig;
  }

  constexpr static CanCoderConfig intakeCanCoderConfig(){

    // ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)
    
    CanCoderConfig intakeCanCoderConfig;
    intakeCanCoderConfig.CanCoderId = 15;
    intakeCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    intakeCanCoderConfig.Offset = IntakeCANCoderOffset;
    intakeCanCoderConfig.absoluteDiscontinuityPoint = 0.05_tr;

    return intakeCanCoderConfig;
  }
};
