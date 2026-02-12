// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"


struct TurretConstants {

  constexpr static const units::turns_per_second_t TurretVelocity = 20.0_tps;
  constexpr static const units::turns_per_second_squared_t TurretAcceleration = 15.0_tr_per_s_sq;
  constexpr static const units::degree_t TurretRangeOfError = 1.5_deg;

  constexpr static const units::degree_t TurretForwardLimit = 360.0_deg; //Poner
  constexpr static const units::degree_t TurretReverseLimit = -360.0_deg; //Poner

  constexpr static const double GearTurretTooth = 100.0;
  constexpr static const double GearEncoder1 = 28.0;
  constexpr static const double GearEncoder2 = 26.0;

  constexpr static const double Slope = (GearEncoder2 * GearEncoder1) / ((GearEncoder1 - GearEncoder2) * GearTurretTooth);
  constexpr static const double GearRatioTurretToEncoder1 = GearTurretTooth / GearEncoder1;
  constexpr static const double GearRatioEncoder1ToTurret = GearEncoder1 / GearTurretTooth;
  
  constexpr static const int TurretMotorId = 23;
  constexpr static const int Turret1CANCoderId = 50;
  constexpr static const int Turret2CANCoderId = 51; //Poner

 constexpr static const OverTalonFXConfig TurretConfig() { //LIMITES QUESTIONABLES *WARNING* CHECAR
        OverTalonFXConfig turretConfig;
        turretConfig.MotorId = TurretMotorId;
        turretConfig.NeutralMode = ControllerNeutralMode::Brake;
        turretConfig.Inverted = false;

        turretConfig.CurrentLimit = 20_A;
        turretConfig.StatorCurrentLimit = 60_A;
        turretConfig.TriggerThreshold = 30_A;
        turretConfig.TriggerThresholdTime = 0.5_s;
        turretConfig.ClosedLoopRampRate = 0.0_s;
        turretConfig.OpenLoopRampRate = 0.05_s;
        // turretConfig.PIDConfigs.WithKV(0.0).WithKP(10.0);

        return turretConfig;
    }

    constexpr static const CanCoderConfig Turret1CANConfig() {
        CanCoderConfig turret1CANConfig;
        turret1CANConfig.CanCoderId = Turret1CANCoderId;
        turret1CANConfig.Offset = 0.0_tr;
        turret1CANConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;

        return turret1CANConfig;
    }

    constexpr static const CanCoderConfig Turret2CANConfig() {
        CanCoderConfig turret2CANConfig;
        turret2CANConfig.CanCoderId = Turret2CANCoderId;
        turret2CANConfig.Offset = 0.0_tr;
        turret2CANConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
        return turret2CANConfig;
    }
};
