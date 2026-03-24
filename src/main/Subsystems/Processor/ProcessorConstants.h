

#pragma once


#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"

struct ProcessorValues {

	units::volt_t spindexer;
	units::volt_t passer;

};

struct ProcessorConstants {

	//Ajustar Voltage
	constexpr static const ProcessorValues Eject{ 5.0_V, 6.0_V };
	constexpr static const ProcessorValues StopProcessor{ 0.0_V, 0.0_V };
	constexpr static const ProcessorValues ReverseProcessor{ -5.0_V, -6.0_V };

	constexpr static const double Indexer1MotorID = 17;
	constexpr static const double Indexer2MotorID = 00;
	constexpr static const double Passer1MotorID = 18;
	constexpr static const double Passer2MotorID = 00;


	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig Indexer1Config() {
		OverTalonFXConfig indexer1Config;
		indexer1Config.MotorId = Indexer1MotorID;
		indexer1Config.NeutralMode = ControllerNeutralMode::Coast;
		indexer1Config.Inverted = false;

		indexer1Config.CurrentLimit = 30_A;
		indexer1Config.StatorCurrentLimit = 120_A;
		indexer1Config.TriggerThreshold = 75_A;
		indexer1Config.TriggerThresholdTime = 0.5_s;
		indexer1Config.ClosedLoopRampRate = 0.0_s;
		indexer1Config.OpenLoopRampRate = 0.1_s;

		return indexer1Config;
	};

	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig Indexer2Config() {
		OverTalonFXConfig indexer2Config;
		indexer2Config.MotorId = Indexer2MotorID;
		indexer2Config.NeutralMode = ControllerNeutralMode::Coast;
		indexer2Config.Inverted = false;

		indexer2Config.CurrentLimit = 30_A;
		indexer2Config.StatorCurrentLimit = 120_A;
		indexer2Config.TriggerThreshold = 75_A;
		indexer2Config.TriggerThresholdTime = 0.5_s;
		indexer2Config.ClosedLoopRampRate = 0.0_s;
		indexer2Config.OpenLoopRampRate = 0.1_s;

		return indexer2Config;
	};


	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig Passer1Config() {
		OverTalonFXConfig passer1Config;
		passer1Config.MotorId = Passer1MotorID;
		passer1Config.NeutralMode = ControllerNeutralMode::Coast;
		passer1Config.Inverted = false;

		passer1Config.CurrentLimit = 30_A;
		passer1Config.StatorCurrentLimit = 60_A;
		passer1Config.TriggerThreshold = 75_A;
		passer1Config.TriggerThresholdTime = 0.5_s;
		passer1Config.ClosedLoopRampRate = 0.0_s;
		passer1Config.OpenLoopRampRate = 0.05_s;

		return passer1Config;
	};

	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig Passer2Config() {
		OverTalonFXConfig passer2Config;
		passer2Config.MotorId = Passer2MotorID;
		passer2Config.NeutralMode = ControllerNeutralMode::Coast;
		passer2Config.Inverted = false;

		passer2Config.CurrentLimit = 30_A;
		passer2Config.StatorCurrentLimit = 60_A;
		passer2Config.TriggerThreshold = 75_A;
		passer2Config.TriggerThresholdTime = 0.5_s;
		passer2Config.ClosedLoopRampRate = 0.0_s;
		passer2Config.OpenLoopRampRate = 0.05_s;

		return passer2Config;
	};

};