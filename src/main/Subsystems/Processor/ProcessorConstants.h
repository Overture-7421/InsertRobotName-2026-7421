

#pragma once

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"

struct processorValues {
	units::volt_t indexerVoltage;
	units::volt_t passerVoltage;
};

struct ProcessorConstants {

	//Ajustar Voltage
	constexpr static const processorValues Eject = { 6.0_V, 8.0_V };
	constexpr static const processorValues Stop = { 0.0_V, 0.0_V };
	constexpr static const processorValues Reverse = { -6.0_V, 0.0_V };

	constexpr static const processorValues Spit = { 3.0_V, 3.0_V };

	constexpr static const double IndexerRightMotorID = 20;
	constexpr static const double PasserUpMotorID = 21;
	constexpr static const double PasserDownMotorID = 29;

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
		OverTalonFXConfig passer2Config;
		passer2Config.MotorId = PasserUpMotorID;
		passer2Config.NeutralMode = ControllerNeutralMode::Coast;
		passer2Config.Inverted = true;

		passer2Config.CurrentLimit = 30_A;
		passer2Config.StatorCurrentLimit = 60_A;
		passer2Config.TriggerThreshold = 75_A;
		passer2Config.TriggerThresholdTime = 0.5_s;
		passer2Config.ClosedLoopRampRate = 0.0_s;
		passer2Config.OpenLoopRampRate = 0.1_s;

		return passer2Config;
	};

	constexpr static const OverTalonFXConfig PasserDownConfig() {
		OverTalonFXConfig passerDownConfig;
		passerDownConfig.MotorId = PasserDownMotorID;
		passerDownConfig.NeutralMode = ControllerNeutralMode::Coast;
		passerDownConfig.Inverted = false;

		passerDownConfig.CurrentLimit = 30_A;
		passerDownConfig.StatorCurrentLimit = 60_A;
		passerDownConfig.TriggerThreshold = 75_A;
		passerDownConfig.TriggerThresholdTime = 0.5_s;
		passerDownConfig.ClosedLoopRampRate = 0.0_s;
		passerDownConfig.OpenLoopRampRate = 0.05_s;

		return passerDownConfig;
	};


};