

#pragma once


#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"

struct ProcessorValues {

	units::volt_t spindexer;
	units::volt_t passer;

};

struct ProcessorConstants {

	//Ajustar Voltage
	constexpr static const ProcessorValues Eject{ 8.0_V, 8.0_V };
	constexpr static const ProcessorValues StopProcessor{ 0.0_V, 0.0_V };
	constexpr static const ProcessorValues ReverseProcessor{ -8.0_V, -8.0_V };

	// constexpr static const ProcessorValues PreloadProcessor{6.0_V, 6.0_V};

	constexpr static const double SpindexerMotorID = 17;
	constexpr static const double PasserMotorID = 18;


	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig SpindexerConfig() {
		OverTalonFXConfig spindexerConfig;
		spindexerConfig.MotorId = SpindexerMotorID;
		spindexerConfig.NeutralMode = ControllerNeutralMode::Coast;
		spindexerConfig.Inverted = false;

		spindexerConfig.CurrentLimit = 30_A;
		spindexerConfig.StatorCurrentLimit = 60_A;
		spindexerConfig.TriggerThreshold = 75_A;
		spindexerConfig.TriggerThresholdTime = 0.5_s;
		spindexerConfig.ClosedLoopRampRate = 0.0_s;
		spindexerConfig.OpenLoopRampRate = 0.1_s;

		return spindexerConfig;
	};


	//Ajustar límites de corriente
	constexpr static const OverTalonFXConfig PasserConfig() {
		OverTalonFXConfig passerConfig;
		passerConfig.MotorId = PasserMotorID;
		passerConfig.NeutralMode = ControllerNeutralMode::Coast;
		passerConfig.Inverted = false;

		passerConfig.CurrentLimit = 30_A;
		passerConfig.StatorCurrentLimit = 60_A;
		passerConfig.TriggerThreshold = 75_A;
		passerConfig.TriggerThresholdTime = 0.5_s;
		passerConfig.ClosedLoopRampRate = 0.0_s;
		passerConfig.OpenLoopRampRate = 0.05_s;

		return passerConfig;
	};

};