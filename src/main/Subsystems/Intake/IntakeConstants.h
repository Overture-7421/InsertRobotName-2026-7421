// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/Sensors/OverCANCoder/OverCANCoder.h>


struct intakeValues {
	units::volt_t rollers;
	units::meter_t intake;
};

struct IntakeConstants {

	constexpr static const intakeValues IntakeOpen{ 7.0_V, 0.0_m }; //Poner todas las posiciones del intake, nada esta puesto bien.
	constexpr static const intakeValues IntakeSustain{ 0.0_V, 0.0_m };

	constexpr static const intakeValues IntakeClose{ 0_V, 0_m };
	constexpr static const units::volt_t RollersStop = 0_V;
	constexpr static const units::volt_t RollersEject = 7.0_V;

	constexpr static const units::meter_t PinionDiameter = 0.0254_m;
	constexpr static const double SensorToMechanism = 3.75;


	constexpr static const units::turns_per_second_t IntakeNormalCruiseVelocity = 7_tps;
	constexpr static const units::turns_per_second_squared_t IntakeNormalCruiseAcceleration = 12_tr_per_s_sq;

	constexpr static const units::turns_per_second_t IntakeLowerCruiseVelocity = 1_tps;
	constexpr static const units::turns_per_second_squared_t IntakeLowerCruiseAcceleration = 1.5_tr_per_s_sq;

	constexpr static const units::meter_t IntakeRangeError = 0_m;

	constexpr static const double SliderCanCoderID = 0;


	constexpr static OverTalonFXConfig sliderRightMotorConfig() {
		OverTalonFXConfig sliderRightMotorConfig;
		sliderRightMotorConfig.MotorId = 15;
		sliderRightMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		sliderRightMotorConfig.Inverted = true;
		sliderRightMotorConfig.useFOC = true;
		sliderRightMotorConfig.PIDConfigs.GravityType = 1;
		sliderRightMotorConfig.PIDConfigs.WithKP(45.0).WithKG(0.5);

		sliderRightMotorConfig.ClosedLoopRampRate = 0.05_s;
		sliderRightMotorConfig.CurrentLimit = 30_A;
		sliderRightMotorConfig.OpenLoopRampRate = 0.0_s;
		sliderRightMotorConfig.StatorCurrentLimit = 120_A;
		sliderRightMotorConfig.TriggerThreshold = 40_A;

		return sliderRightMotorConfig;
	}


	constexpr static OverTalonFXConfig sliderLeftMotorConfig() {
		OverTalonFXConfig sliderLeftMotorConfig;
		sliderLeftMotorConfig.MotorId = 14;
		sliderLeftMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		sliderLeftMotorConfig.Inverted = false;
		sliderLeftMotorConfig.useFOC = true;
		sliderLeftMotorConfig.PIDConfigs.GravityType = 1;
		

		sliderLeftMotorConfig.ClosedLoopRampRate = 0.05_s;
		sliderLeftMotorConfig.CurrentLimit = 30_A;
		sliderLeftMotorConfig.OpenLoopRampRate = 0.0_s;
		sliderLeftMotorConfig.StatorCurrentLimit = 120_A;
		sliderLeftMotorConfig.TriggerThreshold = 40_A;

		return sliderLeftMotorConfig;
	}

	constexpr static OverTalonFXConfig rollersLeftMotorConfig() {
		OverTalonFXConfig rollersLeftMotorConfig;
		rollersLeftMotorConfig.MotorId = 17;
		rollersLeftMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		rollersLeftMotorConfig.Inverted = false;
		rollersLeftMotorConfig.useFOC = true;

		rollersLeftMotorConfig.ClosedLoopRampRate = 0.0_s;
		rollersLeftMotorConfig.CurrentLimit = 20_A;
		rollersLeftMotorConfig.OpenLoopRampRate = 0.1_s;
		rollersLeftMotorConfig.StatorCurrentLimit = 120_A;
		rollersLeftMotorConfig.TriggerThreshold = 30_A;
		rollersLeftMotorConfig.TriggerThresholdTime = 0.5_s;

		return rollersLeftMotorConfig;
	}

		constexpr static OverTalonFXConfig rollersRightMotorConfig() {
		OverTalonFXConfig rollersRightMotorConfig;
		rollersRightMotorConfig.MotorId = 18;
		rollersRightMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		rollersRightMotorConfig.Inverted = false;
		rollersRightMotorConfig.useFOC = true;

		rollersRightMotorConfig.ClosedLoopRampRate = 0.0_s;
		rollersRightMotorConfig.CurrentLimit = 20_A;
		rollersRightMotorConfig.OpenLoopRampRate = 0.1_s;
		rollersRightMotorConfig.StatorCurrentLimit = 120_A;
		rollersRightMotorConfig.TriggerThreshold = 30_A;
		rollersRightMotorConfig.TriggerThresholdTime = 0.5_s;

		return rollersRightMotorConfig;
	}

	constexpr static CanCoderConfig SliderCanCoderConfig() {

		// ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)

		CanCoderConfig SliderCanCoderConfig;
		SliderCanCoderConfig.CanCoderId = 0;
		SliderCanCoderConfig.Offset = 0.0_tr;
		SliderCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
		SliderCanCoderConfig.absoluteDiscontinuityPoint = 0.67_tr;

		return SliderCanCoderConfig;
	}


};
