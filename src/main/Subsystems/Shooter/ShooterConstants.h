// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"


struct ShooterConstants {

	constexpr static const units::turns_per_second_t CruiseVelocity = 90.0_tps;
	constexpr static const units::turns_per_second_squared_t CruiseAcceleration = 205.0_tr_per_s_sq;

	constexpr static const units::turns_per_second_t RangeOfError = 2.0_tps;

	constexpr static const units::turns_per_second_t StopVelocity = 0_tps;

	constexpr static const double SensorToMechanism = 1.0;

	constexpr static const int Motor1ID = 25;
	constexpr static const int Motor2ID = 26;
	constexpr static const int Motor3ID = 27;
	constexpr static const int Motor4ID = 28;


	constexpr static const OverTalonFXConfig Shooter1Config() {
		OverTalonFXConfig shooter1Config;
		shooter1Config.MotorId = Motor1ID;
		shooter1Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter1Config.Inverted = false;
		shooter1Config.useFOC = true; // ??????????????

		shooter1Config.CurrentLimit = 40_A;
		shooter1Config.StatorCurrentLimit = 120_A;
		shooter1Config.TriggerThreshold = 60_A;
		shooter1Config.TriggerThresholdTime = 0.5_s;
		shooter1Config.ClosedLoopRampRate = 0.5_s;
		shooter1Config.PIDConfigs.WithKP(0.255).WithKS(0.0).WithKV(0.12); // 0.05, 0.06, 0.1225

		return shooter1Config;
	}

	constexpr static const OverTalonFXConfig Shooter2Config() {
		OverTalonFXConfig shooter2Config;
		shooter2Config.MotorId = Motor2ID;
		shooter2Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter2Config.Inverted = false;
		shooter2Config.useFOC = true; // ??????????????

		shooter2Config.CurrentLimit = 40_A;
		shooter2Config.StatorCurrentLimit = 120_A;
		shooter2Config.TriggerThreshold = 60_A;
		shooter2Config.TriggerThresholdTime = 0.5_s;
		shooter2Config.ClosedLoopRampRate = 0.5_s;


		return shooter2Config;
	}

	constexpr static const OverTalonFXConfig Shooter3Config() {
		OverTalonFXConfig shooter3Config;
		shooter3Config.MotorId = Motor3ID;
		shooter3Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter3Config.Inverted = false;
		shooter3Config.useFOC = true; // ??????????????

		shooter3Config.CurrentLimit = 40_A;
		shooter3Config.StatorCurrentLimit = 120_A;
		shooter3Config.TriggerThreshold = 60_A;
		shooter3Config.TriggerThresholdTime = 0.5_s;
		shooter3Config.ClosedLoopRampRate = 0.5_s;


		return shooter3Config;
	}

	constexpr static const OverTalonFXConfig Shooter4Config() {
		OverTalonFXConfig shooter4Config;
		shooter4Config.MotorId = Motor4ID;
		shooter4Config.NeutralMode = ControllerNeutralMode::Coast;
		shooter4Config.Inverted = false;
		shooter4Config.useFOC = true; // ??????????????

		shooter4Config.CurrentLimit = 40_A;
		shooter4Config.StatorCurrentLimit = 120_A;
		shooter4Config.TriggerThreshold = 60_A;
		shooter4Config.TriggerThresholdTime = 0.5_s;
		shooter4Config.ClosedLoopRampRate = 0.5_s;


		return shooter4Config;
	}

};