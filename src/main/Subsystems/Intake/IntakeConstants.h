// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/Sensors/OverCANCoder/OverCANCoder.h>


struct intakeValues {
	units::volt_t rollers;
	units::degree_t intake;
};

struct IntakeConstants {

	constexpr static const intakeValues IntakeOpen{ 5_V, 137.0_deg }; //Poner todas las posiciones del intake, nada esta puesto bien.
	constexpr static const intakeValues IntakeGiver{ 4_V, 108.0_deg };
	constexpr static const intakeValues IntakeSustain{ 5_V, 137.0_deg };
	constexpr static const intakeValues IntakeSustainWithoutRollers{ 0_V, 137.0_deg };
	constexpr static const intakeValues IntakeClose{ 0_V, 0_deg };
	constexpr static const intakeValues IntakeInitial{ 0_V, 0_deg };
	constexpr static const units::volt_t RollersStop = 0_V;
	constexpr static const units::volt_t RollersEject = 5_V;


	constexpr static const double intakeRotorToSensor = 15.8667;
	constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 5_tps;
	constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 10_tr_per_s_sq;
	constexpr static const units::degree_t IntakeRangeError = 3_deg;

	constexpr static OverTalonFXConfig intakeMotorConfig() {
		OverTalonFXConfig intakeMotorConfig;
		intakeMotorConfig.MotorId = 24;
		intakeMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		intakeMotorConfig.Inverted = true;
		intakeMotorConfig.useFOC = true;
		intakeMotorConfig.PIDConfigs.GravityType = 1;
		intakeMotorConfig.PIDConfigs.WithKP(45.0).WithKG(0.5);

		intakeMotorConfig.ClosedLoopRampRate = 0.05_s;
		intakeMotorConfig.CurrentLimit = 30_A;
		intakeMotorConfig.OpenLoopRampRate = 0.0_s;
		intakeMotorConfig.StatorCurrentLimit = 120_A;
		intakeMotorConfig.TriggerThreshold = 40_A;

		return intakeMotorConfig;
	}


	constexpr static OverTalonFXConfig intakeSecondMotorConfig() {
		OverTalonFXConfig intakeSecondMotorConfig;
		intakeSecondMotorConfig.MotorId = 14;
		intakeSecondMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		intakeSecondMotorConfig.Inverted = false;
		intakeSecondMotorConfig.useFOC = true;
		intakeSecondMotorConfig.PIDConfigs.GravityType = 1;
		intakeSecondMotorConfig.PIDConfigs.WithKP(45.0).WithKG(0.5);

		intakeSecondMotorConfig.ClosedLoopRampRate = 0.05_s;
		intakeSecondMotorConfig.CurrentLimit = 30_A;
		intakeSecondMotorConfig.OpenLoopRampRate = 0.0_s;
		intakeSecondMotorConfig.StatorCurrentLimit = 120_A;
		intakeSecondMotorConfig.TriggerThreshold = 40_A;

		return intakeSecondMotorConfig;
	}

	constexpr static OverTalonFXConfig rollersMotorConfig() {
		OverTalonFXConfig rollersMotorConfig;
		rollersMotorConfig.MotorId = 16;
		rollersMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		rollersMotorConfig.Inverted = false;
		rollersMotorConfig.useFOC = true;

		rollersMotorConfig.ClosedLoopRampRate = 0.0_s;
		rollersMotorConfig.CurrentLimit = 40_A;
		rollersMotorConfig.OpenLoopRampRate = 0.05_s;
		rollersMotorConfig.StatorCurrentLimit = 120_A;
		rollersMotorConfig.TriggerThreshold = 70_A;
		rollersMotorConfig.TriggerThresholdTime = 0.5_s;

		return rollersMotorConfig;
	}

	constexpr static CanCoderConfig intakeCanCoderConfig() {

		// ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)

		CanCoderConfig intakeCanCoderConfig;
		intakeCanCoderConfig.CanCoderId = 15;
		intakeCanCoderConfig.Offset = 0.13818359375_tr;
		intakeCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
		intakeCanCoderConfig.absoluteDiscontinuityPoint = 0.67_tr;

		return intakeCanCoderConfig;
	}

	constexpr static CanCoderConfig intakeSecondCanCoderConfig() {

		// ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)

		CanCoderConfig intakeCanCoderConfig;
		intakeCanCoderConfig.CanCoderId = 40;
		intakeCanCoderConfig.Offset = 0.0537109375_tr;
		intakeCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
		intakeCanCoderConfig.absoluteDiscontinuityPoint = 0.67_tr;

		return intakeCanCoderConfig;
	}
};
