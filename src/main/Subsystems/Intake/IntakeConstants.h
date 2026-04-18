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

	constexpr static const units::volt_t RollersActive = 7.0_V;
	constexpr static const units::volt_t RollersActiveAuto = 7.0_V;
	constexpr static const units::volt_t RollersStop = 0_V;
	constexpr static const units::meter_t SliderOpen = 0.298_m;
	constexpr static const units::meter_t SliderClose = 0.10_m;
	constexpr static const units::meter_t RollersShouldNotBeMoving = 0.25_m;

	constexpr static const intakeValues IntakeOpen{ RollersActive, SliderOpen }; //Poner todas las posiciones del intake, nada esta puesto bien.
	constexpr static const intakeValues IntakeSustain{ RollersStop, SliderOpen };
	constexpr static const intakeValues IntakeAuto{ RollersActiveAuto, IntakeOpen.intake }; //Poner todas las posiciones del intake, nada esta puesto bien.

	constexpr static const intakeValues IntakeClose{ RollersStop, SliderClose };
	constexpr static const intakeValues IntakeClosing{ RollersActive, SliderClose };
	constexpr static const units::volt_t RollersEject = RollersActive;
	constexpr static const units::volt_t RollersReverse = -RollersActive;

	constexpr static const units::meter_t PinionDiameter = 0.0254_m;
	constexpr static const double SensorToMechanism = 0.166666;
	constexpr static const double RotorToSensor = 38.4; //Dicen que es el doble. Antes 22.5 


	constexpr static const units::turns_per_second_t CruiseVelocity = 24_tps;
	constexpr static const units::turns_per_second_squared_t CruiseAcceleration = 48_tr_per_s_sq;

	constexpr static const units::meter_t IntakeRangeError = 0.04_m;

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
		sliderRightMotorConfig.PIDConfigs.WithKP(30.0).WithKS(0.37); //75 0.42
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
		rollersLeftMotorConfig.CurrentLimit = 50_A;
		rollersLeftMotorConfig.OpenLoopRampRate = 0.1_s;
		rollersLeftMotorConfig.StatorCurrentLimit = 120_A;
		rollersLeftMotorConfig.TriggerThreshold = 50_A;
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
		rollersRightMotorConfig.CurrentLimit = 50_A;
		rollersRightMotorConfig.OpenLoopRampRate = 0.1_s;
		rollersRightMotorConfig.StatorCurrentLimit = 120_A;
		rollersRightMotorConfig.TriggerThreshold = 50_A;
		rollersRightMotorConfig.TriggerThresholdTime = 0.5_s;

		return rollersRightMotorConfig;
	}

	constexpr static CanCoderConfig SliderCanCoderConfig() {

		// ALL REDUCTIONS, LIMITS AND POSITIONS ARE PLACEHOLDERS (TO BE DEFINED)

		CanCoderConfig SliderCanCoderConfig;
		SliderCanCoderConfig.CanCoderId = SliderCanCoderID;
		SliderCanCoderConfig.Offset = 0.078125_tr;
		SliderCanCoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
		SliderCanCoderConfig.absoluteDiscontinuityPoint = 0.75_tr;

		return SliderCanCoderConfig;
	}


};
