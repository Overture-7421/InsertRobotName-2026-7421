// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"


struct ShooterConstants {

  constexpr static const units::turns_per_second_t ShooterCruiseVelocity = 90.0_tps;
  constexpr static const units::turns_per_second_squared_t ShooterCruiseAcceleration = 205.0_tr_per_s_sq;
  constexpr static const units::turns_per_second_t HoodCruiseVelocity = 0.82_tps;
  constexpr static const units::turns_per_second_squared_t HoodCruiseAcceleration = 3.0_tr_per_s_sq;

  constexpr static const units::turns_per_second_t StopShooterVelocity = 0_tps;

  constexpr static const double ShooterSensorToMechanism = 1.0;
  constexpr static const double HoodRotorToSensor = 142.2 ;
  constexpr static const double HoodSensorToMechanism = 17.8;

  constexpr static const int ShooterLeftMotorId = 21;
  constexpr static const int ShooterRightMotorId = 22;

  constexpr static const int HoodMotorId = 19;
  constexpr static const int HoodCANCoderId = 20;


  constexpr static const OverTalonFXConfig ShooterLeftConfig() {
        OverTalonFXConfig shooterLeftConfig;
        shooterLeftConfig.MotorId = ShooterLeftMotorId; //Shooter Down
        shooterLeftConfig.NeutralMode = ControllerNeutralMode::Coast;
        shooterLeftConfig.Inverted = true;
        shooterLeftConfig.useFOC = true; // ??????????????

        shooterLeftConfig.CurrentLimit = 40_A;
        shooterLeftConfig.StatorCurrentLimit = 120_A;
        shooterLeftConfig.TriggerThreshold = 60_A;
        shooterLeftConfig.TriggerThresholdTime = 0.5_s;
        shooterLeftConfig.ClosedLoopRampRate = 0.1_s;
        shooterLeftConfig.PIDConfigs.WithKV(0.117).WithKP(0.2);

        return shooterLeftConfig;
    }

  constexpr static const OverTalonFXConfig ShooterRightConfig() {
        OverTalonFXConfig shooterRightConfig;
        shooterRightConfig.MotorId = ShooterRightMotorId; //Shooter Up
        shooterRightConfig.NeutralMode = ControllerNeutralMode::Coast;
        shooterRightConfig.Inverted = true;
        shooterRightConfig.useFOC = true; // ??????????????

        shooterRightConfig.CurrentLimit = 40_A;
        shooterRightConfig.StatorCurrentLimit = 120_A;
        shooterRightConfig.TriggerThreshold = 60_A;
        shooterRightConfig.TriggerThresholdTime = 0.5_s;
        shooterRightConfig.ClosedLoopRampRate = 0.1_s;
        shooterRightConfig.PIDConfigs.WithKV(0.117).WithKP(0.2);


        return shooterRightConfig;
    }

    constexpr static const OverTalonFXConfig HoodConfig() {
        OverTalonFXConfig hoodConfig;
        hoodConfig.MotorId = HoodMotorId;
        hoodConfig.NeutralMode = ControllerNeutralMode::Brake;
        hoodConfig.useFOC = true;
        hoodConfig.Inverted = false;

        hoodConfig.CurrentLimit = 30_A;
        hoodConfig.StatorCurrentLimit = 120_A;
        hoodConfig.TriggerThreshold = 40_A;
        hoodConfig.TriggerThresholdTime = 0.5_s;
        hoodConfig.ClosedLoopRampRate = 0.05_s;
        hoodConfig.PIDConfigs.WithKV(10.0).WithKP(260.0);

        return hoodConfig;
    }

    constexpr static const CanCoderConfig HoodCANConfig() {
        CanCoderConfig hoodCANConfig;
        hoodCANConfig.CanCoderId = HoodCANCoderId;
        hoodCANConfig.Offset = 0.06298828125_tr;
        hoodCANConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
        return hoodCANConfig;
    }

};