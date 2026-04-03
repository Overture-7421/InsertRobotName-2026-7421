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

	//En teoria solo se puede extender 0.4445 metros maximo
	constexpr static const intakeValues IntakeOpen{ 7.0_V, 0.30_m }; //Poner todas las posiciones del intake, nada esta puesto bien.
	constexpr static const intakeValues IntakeSustain{ 0.0_V, 0.30_m };

	constexpr static const intakeValues IntakeClose{ 0_V, 0.10_m };
	constexpr static const units::volt_t RollersStop = 0_V;
	constexpr static const units::volt_t RollersEject = 7.0_V;
	constexpr static const units::meter_t RollersShouldNotBeMoving = 0.17_m;

	constexpr static const units::meter_t PinionDiameter = 0.0254_m;
	constexpr static const double SensorToMechanism = 0.166666;
	constexpr static const double RotorToSensor = 22.5;


	constexpr static const units::turns_per_second_t CruiseVelocity = 24_tps;
	constexpr static const units::turns_per_second_squared_t CruiseAcceleration = 48_tr_per_s_sq;

	constexpr static const units::meter_t IntakeRangeError = 0.02_m;

	constexpr static const double SliderCanCoderID = 16;


	constexpr static OverTalonFXConfig sliderMotorConfig() {
		OverTalonFXConfig sliderRightMotorConfig;
		sliderRightMotorConfig.MotorId = 15;
		sliderRightMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		sliderRightMotorConfig.Inverted = false;
		sliderRightMotorConfig.useFOC = true;
		sliderRightMotorConfig.PIDConfigs.GravityType = 1;
#ifndef __FRC_ROBORIO__
		sliderRightMotorConfig.PIDConfigs.WithKP(0.6).WithKS(0);
#else
		sliderRightMotorConfig.PIDConfigs.WithKP(75.0).WithKS(0.42);
#endif 


		sliderRightMotorConfig.ClosedLoopRampRate = 0.05_s;
		sliderRightMotorConfig.CurrentLimit = 30_A;
		sliderRightMotorConfig.OpenLoopRampRate = 0.0_s;
		sliderRightMotorConfig.StatorCurrentLimit = 120_A;
		sliderRightMotorConfig.TriggerThreshold = 40_A;

		return sliderRightMotorConfig;
	}


	constexpr static OverTalonFXConfig rollersLeftMotorConfig() {
		OverTalonFXConfig rollersLeftMotorConfig;
		rollersLeftMotorConfig.MotorId = 17;
		rollersLeftMotorConfig.NeutralMode = ControllerNeutralMode::Brake;
		rollersLeftMotorConfig.Inverted = true;
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
		rollersRightMotorConfig.Inverted = true;
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
		SliderCanCoderConfig.CanCoderId = SliderCanCoderID;
		SliderCanCoderConfig.Offset = 0.090087890625_tr;
		SliderCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
		SliderCanCoderConfig.absoluteDiscontinuityPoint = 0.75_tr;

		return SliderCanCoderConfig;
	}


};
