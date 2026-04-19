// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct HoodConstants {

	//27.5 degrees is the maximum

	constexpr static const units::turns_per_second_t CruiseVelocity = 2.0_tps;
	constexpr static const units::turns_per_second_squared_t CruiseAcceleration = 6.0_tr_per_s_sq;

	constexpr static const units::degree_t RangeOfError = 0.5_deg;

	constexpr static const units::degree_t Close = 1.0_deg;

	constexpr static const double RotorToSensor = 18.666666;
	constexpr static const double SensorToMechanism = 6.857142;

	constexpr static const int MotorId = 23;
	constexpr static const int CANCoderId = 24;

	constexpr static const OverTalonFXConfig MotorConfig() {
		OverTalonFXConfig motorConfig;
		motorConfig.MotorId = MotorId;
		motorConfig.NeutralMode = ControllerNeutralMode::Brake;
		motorConfig.useFOC = true;
		motorConfig.Inverted = true;

		motorConfig.CurrentLimit = 30_A;
		motorConfig.StatorCurrentLimit = 120_A;
		motorConfig.TriggerThreshold = 40_A;
		motorConfig.TriggerThresholdTime = 0.5_s;
		motorConfig.ClosedLoopRampRate = 0.05_s;
		motorConfig.PIDConfigs.WithKP(400.0).WithKI(30.0).WithKV(10.8); // 375, 30, 9

		return motorConfig;
	}

	constexpr static const CanCoderConfig CANCoderConfig() {
		CanCoderConfig canCoderConfig;
		canCoderConfig.CanCoderId = CANCoderId;
#ifndef __FRC_ROBORIO__
		canCoderConfig.Offset = 0_tr;
#else
		canCoderConfig.Offset = -0.781982421875_tr;
#endif 
		canCoderConfig.absoluteDiscontinuityPoint = 0.95_tr;
		canCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
		return canCoderConfig;
	}
};
