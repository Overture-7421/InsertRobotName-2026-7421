// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"
#include "IntakeConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {

	sliderMotor.setRotorToSensorRatio(IntakeConstants::RotorToSensor);
	sliderMotor.setSensorToMechanism(IntakeConstants::SensorToMechanism);
	sliderMotor.setFusedCANCoder(IntakeConstants::SliderCanCoderConfig().CanCoderId);

	sliderMotor.configureMotionMagic(IntakeConstants::CruiseVelocity, IntakeConstants::CruiseAcceleration, 0.0_tr_per_s_cu);

	rollersLeftMotor.setFollow(rollersRightMotor.GetDeviceID(),true);

}

void Intake::setRollersVoltage(units::volt_t targetVoltage) {
	rollersRightMotor.SetControl(rollersVoltage.WithOutput(targetVoltage).WithEnableFOC(true));
}

units::turn_t Intake::transformMetersToTurns(units::meter_t distance){
	return units::turn_t(distance.value()/(IntakeConstants::PinionDiameter.value()*M_PI));	
}

units::meter_t Intake::transformTurnsToMeters(units::turn_t angle){
	return units::meter_t (angle.value()*(IntakeConstants::PinionDiameter.value()*M_PI));
}

bool Intake::intakeReached(units::meter_t targetDistance) {
	units::meter_t intakeError = targetDistance - transformTurnsToMeters(sliderMotor.GetPosition().GetValue());
	return (units::math::abs(intakeError) < IntakeConstants::IntakeRangeError);
}

void Intake::setIntakeDistance(units::meter_t targetDistance) {
	units::turn_t targetAngle = transformMetersToTurns(targetDistance);
	sliderMotor.SetControl(intakeVoltage.WithPosition(targetAngle).WithEnableFOC(true));
}

units::meter_t Intake::getIntakePosition() {
	return transformTurnsToMeters(sliderMotor.GetPosition().GetValue());
}


frc2::CommandPtr Intake::setIntakeCmd(intakeValues targetPos) {
	return frc2::cmd::Sequence(setSliderCmd(targetPos.intake), setRollersCmd(targetPos.rollers));
}

frc2::CommandPtr Intake::setIntakeSlowModeCmd(intakeValues targetPos) {
	return frc2::FunctionalCommand(
		[this, targetPos]() {
		setRollersVoltage(targetPos.rollers);
		auto currentMeters = getIntakePosition();
		intakeSlowModeFilter.Reset(currentMeters);
	},

		[this, targetPos]() {
		setIntakeDistance(intakeSlowModeFilter.Calculate(targetPos.intake));

	},

	[this](bool interrupted) {},

	[this, targetPos] {
		return (intakeReached(targetPos.intake));
	}, {this}
	).ToPtr();
}

frc2::CommandPtr Intake::setSliderCmd(units::meter_t targetDistance) {
	return frc2::FunctionalCommand(
		[this, targetDistance]() {
		setIntakeDistance(targetDistance);
	},

		[]() {
	},

	[this](bool interrupted) {},

	[this, targetDistance] {
		return (intakeReached(targetDistance));
	}, {this}
	).ToPtr();}

frc2::CommandPtr Intake::setIntakeCharacterization(units::meter_t distance, units::volt_t voltage) {
	return frc2::cmd::Sequence(
		setSliderCmd(distance),
		setRollersCmd(voltage)
	);
}

frc2::CommandPtr Intake::setRollersCmd(units::volt_t targetVoltage) {
	return frc2::FunctionalCommand(
		[this, targetVoltage]() {
		setRollersVoltage(targetVoltage);
	},

		[]() {
	},

	[](bool interrupted) {},

	[] {
		return true;
	}, {this}
	).ToPtr();
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::UpdateTelemetry() {
	frc::SmartDashboard::PutNumber("Intake/Current", transformTurnsToMeters(units::turn_t(sliderMotor.GetPosition().GetValue().value())).value());

	double targetDistance = sliderMotor.GetClosedLoopReference().GetValue();
	frc::SmartDashboard::PutNumber("Intake/ErrorAngle", sliderMotor.GetClosedLoopError().GetValue());
	frc::SmartDashboard::PutNumber("Intake/TargetAngle", transformTurnsToMeters(units::turn_t(targetDistance)).value());
	frc::SmartDashboard::PutBoolean("Intake/isIntakeAtAngle", intakeReached(units::meter_t(targetDistance)));
}