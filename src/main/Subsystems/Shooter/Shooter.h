// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ShooterConstants.h"
#include "Constants.h"
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <units/math.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ShooterState.h"
#include <wpi/circular_buffer.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/voltage.h>

class Shooter : public frc2::SubsystemBase {
public:

	Shooter();
	void setObjectiveVelocity(units::turns_per_second_t velocity);
	units::turns_per_second_t getShooterVelocity();
	bool isShooterAtVelocity();
	ShooterState getState();
	/**
	* Whether to keep Holding state, even though we may not be at the target. This is useful when continously shooting fuel, as they slow down the flywheel.
	* "Hold" instructs the Shooter to keep Holding PID Slot (No PID gains, only kS,kV,kA) once we initially reach Holding state.
	* "Release" allows Holding state to transition to WindUp if we are no longer at the target. Windup returns to "normal" slot (Both PID gains and kS,kV,kA).
	*/
	void Hold();
	void Release();

	frc2::CommandPtr setShooterVelocityCmd(units::turns_per_second_t velocity);
	void UpdateTelemetry();
	void Periodic() override;

	frc2::CommandPtr sysIdQuasistatic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Quasistatic(direction);
	}

	frc2::CommandPtr sysIdDynamic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Dynamic(direction);
	}

private:

	OverTalonFX shooterLeftUpMotor{ ShooterConstants::ShooterLeftUpConfig(), robotConstants::rio };
	OverTalonFX shooterLeftDownMotor{ ShooterConstants::ShooterLeftDownConfig(), robotConstants::rio };
	OverTalonFX shooterRightUpMotor{ ShooterConstants::ShooterRightUpConfig(), robotConstants::rio };
	OverTalonFX shooterRightDownMotor{ ShooterConstants::ShooterRightDownConfig(), robotConstants::rio };

	ctre::phoenix6::controls::MotionMagicVelocityVoltage shooterVoltageRequest{ 0.0_tps };
	units::turns_per_second_t targetVelocity = 0_tps;

	units::second_t lastTimeOnTarget = 0_s;
	ShooterState state = ShooterState::WindUp;
	bool shouldHold = false;

	wpi::circular_buffer<double> kVEstimator{ ShooterConstants::HoldingSamples };
	double averagekV = 0;
	int currentPIDSlot = 0;

	ctre::phoenix6::configs::TalonFXConfiguration shooterLeftCTREConfig;

	frc2::sysid::SysIdRoutine sysIdRoutine{
	frc2::sysid::Config{0.5_V / 1_s, 3.5_V, 10_s, {}},
	frc2::sysid::Mechanism{
		[this](units::volt_t driveVoltage) {
			shooterLeftUpMotor.SetVoltage(driveVoltage);
		},
		[this](frc::sysid::SysIdRoutineLog* log) {

		log->Motor("Shooter")
			.voltage(shooterLeftUpMotor.GetMotorVoltage().GetValue())
			.position(shooterLeftUpMotor.GetPosition().GetValue())
			.velocity(shooterLeftUpMotor.GetVelocity().GetValue());
		},
		this} };

};
