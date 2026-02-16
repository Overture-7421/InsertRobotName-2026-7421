

#pragma once


#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"

struct ProcessorValues {

    units::volt_t spindexer;
    units::volt_t passer;

};

struct ProcessorConstants{

//Ajustar Voltage
constexpr static const ProcessorValues StartProcessor{0.0_V, 0.0_V};
constexpr static const ProcessorValues StopProcessor{0.0_V, 0.0_V};
constexpr static const ProcessorValues ReverseProcessor{0.0_V, 0.0_V};

constexpr static const double SpindexerMotorID = 17;
constexpr static const double PasserMotorID = 18;


//Ajustar límites de corriente
constexpr static const OverTalonFXConfig SpindexerConfig(){
        OverTalonFXConfig spindexerConfig;
        spindexerConfig.MotorId = SpindexerMotorID;
        spindexerConfig.NeutralMode = ControllerNeutralMode::Coast;
        spindexerConfig.Inverted = false;

        spindexerConfig.CurrentLimit = 0_A;
        spindexerConfig.StatorCurrentLimit = 0_A;
        spindexerConfig.TriggerThreshold = 0_A;
        spindexerConfig.TriggerThresholdTime = 0_s;
        spindexerConfig.ClosedLoopRampRate = 0_s;
        spindexerConfig.OpenLoopRampRate = 0_s;

        return spindexerConfig;
};


//Ajustar límites de corriente
constexpr static const OverTalonFXConfig PasserConfig(){
        OverTalonFXConfig passerConfig;
        passerConfig.MotorId = PasserMotorID;
        passerConfig.NeutralMode = ControllerNeutralMode::Coast;
        passerConfig.Inverted = false;

        passerConfig.CurrentLimit = 0_A;
        passerConfig.StatorCurrentLimit = 0_A;
        passerConfig.TriggerThreshold = 0_A;
        passerConfig.TriggerThresholdTime = 0_s;
        passerConfig.ClosedLoopRampRate = 0_s;
        passerConfig.OpenLoopRampRate = 0_s;

        return passerConfig;
};

};