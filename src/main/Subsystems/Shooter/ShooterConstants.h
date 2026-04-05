// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"


struct ShooterConstants {

	constexpr static const units::turns_per_second_t CruiseVelocity = 90.0_tps;
	constexpr static const units::turns_per_second_squared_t CruiseAcceleration = 400.0_tr_per_s_sq;
	constexpr static const units::turns_per_second_cubed_t CruiseJerk = 4000.0_tr_per_s_cu;

	constexpr static const units::turns_per_second_t RangeOfError = 0.30_tps;

	constexpr static const units::turns_per_second_t StopVelocity = 0_tps;

	constexpr static const double SensorToMechanism = 1.0;

	constexpr static const int MotorLeftUpID = 25;
	constexpr static const int MotorLeftDownID = 26;
	constexpr static const int MotorRightUpID = 27;
	constexpr static const int MotorRightDownID = 28;

	constexpr static const OverTalonFXConfig ShooterLeftUpConfig() {
		OverTalonFXConfig shooter1Config;
		shooter1Config.MotorId = MotorLeftUpID;
		shooter1Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter1Config.Inverted = true;
		shooter1Config.useFOC = true; // ??????????????

		shooter1Config.CurrentLimit = 40_A;
		shooter1Config.StatorCurrentLimit = 120_A;
		shooter1Config.TriggerThreshold = 60_A;
		shooter1Config.TriggerThresholdTime = 0.5_s;
		shooter1Config.ClosedLoopRampRate = 0.0_s;
#ifndef __FRC_ROBORIO__
		shooter1Config.PIDConfigs.WithKP(0.015).WithKS(0.0).WithKV(0.1195);
#else
		shooter1Config.PIDConfigs.WithKP(0.0).WithKS(0.0).WithKV(0.0); //0.015, 0.313, 0.1215
#endif 

		return shooter1Config;
	}

	constexpr static const OverTalonFXConfig ShooterLeftDownConfig() {
		OverTalonFXConfig shooter2Config;
		shooter2Config.MotorId = MotorLeftDownID;
		shooter2Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter2Config.Inverted = true;
		shooter2Config.useFOC = true; // ??????????????

		shooter2Config.CurrentLimit = 40_A;
		shooter2Config.StatorCurrentLimit = 120_A;
		shooter2Config.TriggerThreshold = 60_A;
		shooter2Config.TriggerThresholdTime = 0.5_s;
		shooter2Config.ClosedLoopRampRate = 0.0_s;


		return shooter2Config;
	}

	constexpr static const OverTalonFXConfig ShooterRightUpConfig() {
		OverTalonFXConfig shooter3Config;
		shooter3Config.MotorId = MotorRightUpID;
		shooter3Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter3Config.Inverted = false;
		shooter3Config.useFOC = true; // ??????????????

		shooter3Config.CurrentLimit = 40_A;
		shooter3Config.StatorCurrentLimit = 120_A;
		shooter3Config.TriggerThreshold = 60_A;
		shooter3Config.TriggerThresholdTime = 0.5_s;
		shooter3Config.ClosedLoopRampRate = 0.0_s;


		return shooter3Config;
	}

	constexpr static const OverTalonFXConfig ShooterRightDownConfig() {
		OverTalonFXConfig shooter4Config;
		shooter4Config.MotorId = MotorRightDownID;
		shooter4Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter4Config.Inverted = false;
		shooter4Config.useFOC = true; // ??????????????

		shooter4Config.CurrentLimit = 40_A;
		shooter4Config.StatorCurrentLimit = 120_A;
		shooter4Config.TriggerThreshold = 60_A;
		shooter4Config.TriggerThresholdTime = 0.5_s;
		shooter4Config.ClosedLoopRampRate = 0.0_s;


		return shooter4Config;
	}

};