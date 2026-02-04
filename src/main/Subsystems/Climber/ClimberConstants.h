#pragma once
#include <units/length.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
//#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>


struct ClimberConstants {

    constexpr static const units::volt_t feedForward = 0_V;
    constexpr static const units::meter_t RangeError = 0.04_m; //Range of error the climber is permiited to have.

    

    constexpr static const units::turns_per_second_t ClimberCruiseVelocity = 120.0_tps; //The velocity at which the climber travels
    constexpr static const units::turns_per_second_squared_t ClimberUpperCruiseAcceleration = 75_tr_per_s_sq; //The acceleration the climber gains when going up
    constexpr static const units::turns_per_second_squared_t ClimberLowerCruiseAcceleration = 20_tr_per_s_sq; //The acceleration the climber gains when going down

    constexpr static const double LowerSensorToMechanism = 5.6; //The gear ratio there exists between the encoder to the actual mechanism.
    constexpr static const units::meter_t Diameter = 0.07366_m; //Diameter of the "pulley" neeeded for the climber

    //Configuration of the motors the climber uses
    constexpr static const OverTalonFXConfig RightConfig() {
        OverTalonFXConfig right;
        right.MotorId = 21;
        right.NeutralMode = ControllerNeutralMode::Brake;
        right.Inverted = true;
        right.useFOC = true;
        right.CurrentLimit = 25_A;
        right.StatorCurrentLimit = 120_A;
        right.TriggerThreshold = 40_A;
        right.TriggerThresholdTime = 0.5_s;
        right.ClosedLoopRampRate = 0.05_s;

        return right;
    }

    constexpr static const OverTalonFXConfig LeftConfig() {
        OverTalonFXConfig left;
        left.MotorId = 20;
        left.NeutralMode = ControllerNeutralMode::Brake;
        left.Inverted = false;
        left.useFOC = true;
        left.PIDConfigs.WithKG(0.18).WithKS(0.4).WithKP(30); //G=0.37   S=0.5  P=20
        left.CurrentLimit = 25_A;
        left.StatorCurrentLimit = 120_A;
        left.TriggerThreshold = 40_A;
        left.TriggerThresholdTime = 0.5_s;
        left.ClosedLoopRampRate = 0.05_s;

        return left;
    }

    constexpr static const units::meter_t diameter = 2.54_m; // falta preguntar cual va ser el diametro del engrane
};