

#pragma once


#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"

struct ProcessorValues {

	units::volt_t spindexer;
	units::volt_t passer;

};

struct ProcessorConstants {

	//Ajustar Voltage
	constexpr static const ProcessorValues Eject{ 6.0_V, 4.0_V };
	constexpr static const ProcessorValues StopProcessor{ 0.0_V, 0.0_V };
	constexpr static const ProcessorValues ReverseProcessor{ -6.0_V, -4.0_V };

	constexpr static const double IndexerRightMotorID = 20;
	constexpr static const double PasserUpMotorID = 21;
	constexpr static const double PasserDownMotorID = 22;

	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig IndexerRightConfig() {
		OverTalonFXConfig indexer2Config;
		indexer2Config.MotorId = IndexerRightMotorID;
		indexer2Config.NeutralMode = ControllerNeutralMode::Coast;
		indexer2Config.Inverted = true;

		indexer2Config.CurrentLimit = 30_A;
		indexer2Config.StatorCurrentLimit = 120_A;
		indexer2Config.TriggerThreshold = 75_A;
		indexer2Config.TriggerThresholdTime = 0.5_s;
		indexer2Config.ClosedLoopRampRate = 0.0_s;
		indexer2Config.OpenLoopRampRate = 0.1_s;

		return indexer2Config;
	};


	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig PasserUpConfig() {
		OverTalonFXConfig passer1Config;
		passer1Config.MotorId = PasserUpMotorID;
		passer1Config.NeutralMode = ControllerNeutralMode::Coast;
		passer1Config.Inverted = true;

		passer1Config.CurrentLimit = 30_A;
		passer1Config.StatorCurrentLimit = 60_A;
		passer1Config.TriggerThreshold = 75_A;
		passer1Config.TriggerThresholdTime = 0.5_s;
		passer1Config.ClosedLoopRampRate = 0.0_s;
		passer1Config.OpenLoopRampRate = 0.05_s;

		return passer1Config;
	};

	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig PasserDownConfig() {
		OverTalonFXConfig passer2Config;
		passer2Config.MotorId = PasserDownMotorID;
		passer2Config.NeutralMode = ControllerNeutralMode::Coast;
		passer2Config.Inverted = true;

		passer2Config.CurrentLimit = 30_A;
		passer2Config.StatorCurrentLimit = 60_A;
		passer2Config.TriggerThreshold = 75_A;
		passer2Config.TriggerThresholdTime = 0.5_s;
		passer2Config.ClosedLoopRampRate = 0.0_s;
		passer2Config.OpenLoopRampRate = 0.05_s;

		return passer2Config;
	};

};