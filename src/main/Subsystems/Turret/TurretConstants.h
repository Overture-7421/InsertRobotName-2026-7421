// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"


struct TurretConstants {

  constexpr static const units::turns_per_second_t TurretVelocity = 2.5_tps;
  constexpr static const units::turns_per_second_squared_t TurretAcceleration = 7.5_tr_per_s_sq;
  constexpr static const units::degree_t TurretRangeOfError = 1.5_deg;

  constexpr static const double ChassisAngularVelocityCompensator = 1.0; //Poner
  constexpr static const double CableSpringConstant = 0.479; //Poner

  constexpr static const double SensorToMechanism = 50.0;

  constexpr static const units::degree_t TurretForwardLimit = 202.0_deg; //Poner
  constexpr static const units::degree_t TurretReverseLimit = -219.0_deg; //Poner

  constexpr static const units::degree_t TurretSafetyZoneCablesForward = 53.0_deg;
  constexpr static const units::degree_t TurretSafetyZoneCablesReverse = -47.0_deg;
//   constexpr static const units::degree_t TurretForceAppliedRange = TurretForwardLimit - TurretSafetyZoneCables;

  constexpr static const double GearTurretTooth = 100.0;
  constexpr static const double GearEncoder1 = 28.0;
  constexpr static const double GearEncoder2 = 26.0;

  constexpr static const double Slope = (GearEncoder2 * GearEncoder1) / ((GearEncoder1 - GearEncoder2) * GearTurretTooth);
  constexpr static const double GearRatioTurretToEncoder1 = GearTurretTooth / GearEncoder1;
  constexpr static const double GearRatioEncoder1ToTurret = GearEncoder1 / GearTurretTooth;
  
  constexpr static const int TurretMotorId = 23;
  constexpr static const int Turret1CANCoderId = 24;
  constexpr static const int Turret2CANCoderId = 28; //Poner

 constexpr static const OverTalonFXConfig TurretConfig() { //LIMITES QUESTIONABLES *WARNING* CHECAR
        OverTalonFXConfig turretConfig;
        turretConfig.MotorId = TurretMotorId;
        turretConfig.NeutralMode = ControllerNeutralMode::Brake;
        turretConfig.Inverted = false;

        turretConfig.CurrentLimit = 40_A;
        turretConfig.StatorCurrentLimit = 120_A;
        turretConfig.TriggerThreshold = 60_A;
        turretConfig.TriggerThresholdTime = 0.5_s;
        turretConfig.ClosedLoopRampRate = 0.05_s;
        turretConfig.OpenLoopRampRate = 0.0_s;
        turretConfig.PIDConfigs.WithKP(150.0).WithKS(0.279); // 6 y 18

        return turretConfig;
    }

    constexpr static const CanCoderConfig Turret1CANConfig() {
        CanCoderConfig turret1CANConfig;
        turret1CANConfig.CanCoderId = Turret1CANCoderId;
        turret1CANConfig.Offset = -0.06298828125_tr;
        turret1CANConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
        turret1CANConfig.absoluteDiscontinuityPoint = 1.0_tr;

        return turret1CANConfig;
    }

    constexpr static const CanCoderConfig Turret2CANConfig() {
        CanCoderConfig turret2CANConfig;
        turret2CANConfig.CanCoderId = Turret2CANCoderId;
        turret2CANConfig.Offset = -0.06005859375_tr;
        turret2CANConfig.absoluteDiscontinuityPoint = 1.0_tr;
        return turret2CANConfig;
    }
};
