// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "IntakeConstants.h"
#include "Constants.h"
#include <frc2/command/Commands.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>


class Intake : public frc2::SubsystemBase {
public:
	units::turn_t transformMetersToTurns(units::meter_t distance);
	units::meter_t transformTurnsToMeters(units::turn_t angle);
	Intake();


	void setRollersVoltage(units::volt_t targetVoltage);
	void setIntakeDistance(units::meter_t targetDistance);
	bool intakeReached(units::meter_t targetDistance);
	units::meter_t getIntakePosition();


	frc2::CommandPtr setIntakeCmd(intakeValues targetPos); //To Open
	frc2::CommandPtr setIntakeSlowModeCmd(intakeValues targetPos); //It was just for prove logic
	frc2::CommandPtr setRollersCmd(units::volt_t targetVoltage); // Just Rollers
	frc2::CommandPtr setSliderCmd(units::meter_t targetDistance); //Just Slider
	frc2::CommandPtr setIntakeClosingCmd(intakeValues targetPos); //To Close (Because of the range)

	frc2::CommandPtr setIntakeCharacterization(units::meter_t distance, units::volt_t voltage);

	void UpdateTelemetry();

	void Periodic() override;

	frc::SlewRateLimiter<units::length::meter> intakeSlowModeFilter{ 0.13_mps }; //0.1 m/s de velocidad máxima de cambio

private:
	OverTalonFX sliderMotor{ IntakeConstants::sliderMotorConfig(), robotConstants::rio };
	OverTalonFX rollersRightMotor{ IntakeConstants::rollersRightMotorConfig(), robotConstants::rio };
	OverTalonFX rollersLeftMotor{ IntakeConstants::rollersLeftMotorConfig(), robotConstants::rio };
	OverCANCoder sliderRightCanCoder{ IntakeConstants::SliderCanCoderConfig(), robotConstants::rio };

	ctre::phoenix6::controls::MotionMagicVoltage intakeVoltage{ 0_tr };
	ctre::phoenix6::controls::VoltageOut rollersVoltage{ 0_V };

	units::volt_t targetRollerVoltage{ 0_V };

};
